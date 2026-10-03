# Copyright (c) Meta Platforms, Inc. and affiliates.
# SPDX-License-Identifier: Apache-2.0

"""Exercise the AXP192 PEK tap-to-talk decoder behind the Core2 talk button."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "axp192_pek.h"

static axp192_pek_t key;

static void expect(unsigned got, unsigned want, const char *what)
{
    if (got != want) {
        printf("FAIL %s: got %u want %u\n", what, got, want);
        assert(got == want);
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "tap_talk")) {
        /* A tap starts recording, the next sends it. */
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 100), AXP192_PEK_PRESS, "start");
        expect(axp192_pek_poll(&key, 0, 200), 0, "recording");
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 300), AXP192_PEK_RELEASE, "send");
        expect(axp192_pek_poll(&key, 0, 400), 0, "idle");
    } else if (!strcmp(argv[1], "timeout")) {
        /* A lone press auto-releases after 1500 ticks (~15 s). */
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 0), AXP192_PEK_PRESS, "start");
        expect(axp192_pek_poll(&key, 0, 1499), 0, "held");
        expect(axp192_pek_poll(&key, 0, 1500), AXP192_PEK_RELEASE, "auto release");
        expect(axp192_pek_poll(&key, 0, 1600), 0, "idle");
    } else if (!strcmp(argv[1], "tap_at_timeout")) {
        /* A tap landing exactly on the timeout still just sends. */
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 0), AXP192_PEK_PRESS, "start");
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 1500), AXP192_PEK_RELEASE, "send");
        expect(axp192_pek_poll(&key, 0, 1501), 0, "idle");
    } else if (!strcmp(argv[1], "hold")) {
        /* A long hold's trailing short lands as down-then-up, which Muse
         * answers with "hold longer to talk". */
        expect(axp192_pek_poll(&key, AXP192_PEK_LONG | AXP192_PEK_SHORT, 500),
               AXP192_PEK_PRESS | AXP192_PEK_RELEASE, "hold pulse");
        expect(axp192_pek_poll(&key, 0, 501), 0, "idle");
    } else if (!strcmp(argv[1], "long_ignored")) {
        /* A long press while recording changes nothing. */
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 0), AXP192_PEK_PRESS, "start");
        expect(axp192_pek_poll(&key, AXP192_PEK_LONG, 100), 0, "long ignored");
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 200), AXP192_PEK_RELEASE, "send");
    } else if (!strcmp(argv[1], "wrap")) {
        /* The timeout survives the 32-bit tick wrapping. */
        expect(axp192_pek_poll(&key, AXP192_PEK_SHORT, 0xFFFFFF00u), AXP192_PEK_PRESS, "start");
        expect(axp192_pek_poll(&key, 0, 0x4DBu), 0, "held");
        expect(axp192_pek_poll(&key, 0, 0x4DCu), AXP192_PEK_RELEASE, "auto release");
    } else {
        assert(!"unknown scenario");
    }
    return 0;
}
'''


class Axp192PekTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        tmp = Path(cls.temp.name)
        (tmp / "test.c").write_text(HARNESS)
        cls.exe = tmp / ("test.exe" if os.name == "nt" else "test")
        board = ROOT / "components/muse/boards"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(board),
            str(board / "axp192_pek.c"), str(tmp / "test.c"), "-o", str(cls.exe)
        ], check=True, capture_output=True)

    def test_tap_to_talk_sequences(self):
        for scenario in ("tap_talk", "timeout", "tap_at_timeout", "hold", "long_ignored", "wrap"):
            with self.subTest(scenario=scenario):
                subprocess.run([str(self.exe), scenario], check=True, capture_output=True, timeout=10)
