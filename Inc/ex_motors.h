/*******************************************************************************
 *
 *   EMERGENCE PROJECT (http://emergencejourney.org)
 *   Copyright (C) 2018 - David Marchaland (david.marchaland@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/


#pragma once

void EX_MOTORS_Init(void);
void EX_MOTORS_Setup(void);
void EX_MOTORS_Enable(void);
void EX_MOTORS_Disable(void);

void EX_MOTORS_SetMotion(uint64_t timestamp, int32_t position, int32_t velocity, uint32_t channel);

void EX_MOTORS_RunTestMode(_Bool loop, uint32_t delay);
