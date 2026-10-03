/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * AXP192 PEK (power-key) decoding for tap-to-talk. The PEK short/long-press
 * IRQs only fire on release, and the edge IRQs never fire at all (observed on
 * hardware), so hold-to-talk is impossible: a tap toggles recording instead.
 * The first tap starts it, the next sends it, and a lone press auto-releases
 * after 15 s so a missed second tap can't wedge Muse recording. A long hold's
 * trailing short lands as down-then-up, which Muse answers with "hold longer
 * to talk". Dependency-free so the host tests can drive it; register numbers
 * from M5Unified's AXP192 driver and lewisxhe/AXP202X_Library.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Latched INTSTS3 (0x46) PEK bits. */
#define AXP192_PEK_SHORT (1u << 1)
#define AXP192_PEK_LONG (1u << 0)

/* Talk-button transitions for one poll. */
#define AXP192_PEK_PRESS (1u << 0)
#define AXP192_PEK_RELEASE (1u << 1)

/* A press auto-releases after this many poll ticks (~10 ms each). */
#define AXP192_PEK_AUTO_RELEASE_TICKS 1500u

typedef struct {
    bool down;
    uint32_t down_tick;
} axp192_pek_t;

/*
 * Fold one latched short/long-status read into talk-button transitions.
 * `tick` is the poll counter and may wrap.
 */
unsigned axp192_pek_poll(axp192_pek_t *key, uint8_t status, uint32_t tick);
