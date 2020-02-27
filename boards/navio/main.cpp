/*
 * Copyright (c) 2020, TBD
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "navio_board.h"
#include "rosflight.h"
#include "mavlink.h"
#include <stdio.h>

//#pragma GCC diagnostic ignored "-Wmissing-field-initializers" //Because this was unnecessary and annoying

uint32_t error_count_ = 0;//Used for counting resets
rosflight_firmware::ROSflight *rosflight=nullptr;//Used to access important variables in case of a hard fault
rosflight_firmware::StateManager::State get_state()//Used in case of a hard fault
{
  if (rosflight==nullptr)
  {
#pragma GCC diagnostic push //Ignore blank fields in struct
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    rosflight_firmware::StateManager::State ret = {0};
#pragma GCC diagnostic pop
    return ret;
  }
  return rosflight->state_manager_.state();
}

int main()
{
  printf("hello from the other side\n");
  rosflight_firmware::NavioBoard board;
  board.init_board();
  rosflight_firmware::Mavlink mavlink(board);
  rosflight_firmware::ROSflight firmware(board, mavlink);

  firmware.init();

//  while (true)
//  {
//    firmware.run();
//  }
  return 0;
}
