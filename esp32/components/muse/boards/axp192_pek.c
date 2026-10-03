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

#include "axp192_pek.h"

unsigned axp192_pek_poll(axp192_pek_t *key, uint8_t status, uint32_t tick)
{
    unsigned out = 0;
    status &= AXP192_PEK_SHORT | AXP192_PEK_LONG;
    if ((status & AXP192_PEK_LONG) && !key->down) {
        key->down = true;
        key->down_tick = tick;
        out |= AXP192_PEK_PRESS;
    }
    if (status & AXP192_PEK_SHORT) {
        if (key->down) {
            key->down = false;
            out |= AXP192_PEK_RELEASE;
        } else {
            key->down = true;
            key->down_tick = tick;
            out |= AXP192_PEK_PRESS;
        }
    }
    if (key->down && tick - key->down_tick >= AXP192_PEK_AUTO_RELEASE_TICKS) {
        key->down = false;
        out |= AXP192_PEK_RELEASE;
    }
    return out;
}
