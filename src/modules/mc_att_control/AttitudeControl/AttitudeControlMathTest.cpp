/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "AttitudeControlMath.hpp"

using namespace matrix;
using namespace AttitudeControlMath;

static const Vector3f z_unit(0.f, 0.f, 1.f);

TEST(AttitudeControlMath, tiltCorrectionNoError)
{
	// 给定：一个尚未叠加偏航旋转的倾斜目标
	Quatf q_tilt_sp_ne(z_unit, Vector3f(-0.3, 0.1, 0.7));

	// 以及：一个偏航目标
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -1.23f);

	// 当：当前偏航误差为零（倾斜状态不限）
	const Quatf q = q_sp_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// 则：倾斜目标保持不变
	EXPECT_TRUE(isEqual(q_tilt_sp_ne_before, q_tilt_sp_ne));
}

TEST(AttitudeControlMath, tiltCorrectionYaw180)
{
	// 给定：一个尚未叠加偏航旋转的倾斜目标，以及一个偏航目标
	Quatf q_tilt_sp_ne(z_unit, Vector3f(-0.3, 0.1, 0.7));
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -M_PI_F / 2.f);

	// 当：存在 180 度的偏航误差
	const Quatf q_yaw = Quatf(AxisAnglef(z_unit, M_PI_F / 2.f));
	const Quatf q = q_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// 则：倾斜方向反转（修正后的倾斜角度相同，但旋转轴方向相反）
	EXPECT_FLOAT_EQ(AxisAnglef(q_tilt_sp_ne_before).angle(), AxisAnglef(q_tilt_sp_ne).angle());
	EXPECT_TRUE(isEqual(AxisAnglef(q_tilt_sp_ne_before).axis(), -AxisAnglef(q_tilt_sp_ne).axis()));
}

TEST(AttitudeControlMath, tiltCorrection)
{
	// 给定：一个尚未叠加偏航旋转的倾斜目标，以及一个偏航目标
	Quatf q_tilt_sp_ne(z_unit, Vector3f(0.5, -0.1, 0.7));
	const Quatf q_sp_yaw = AxisAnglef(z_unit, -1.23f);

	// 当：存在一定的偏航误差
	const Quatf q_yaw = Quatf(AxisAnglef(z_unit, 3.1f));
	const Quatf q = q_yaw * Quatf(z_unit, Vector3f(0.1f, -0.2f, 1.f));
	const Quatf q_tilt_sp_ne_before = q_tilt_sp_ne;
	correctTiltSetpointForYawError(q_tilt_sp_ne, q, q_sp_yaw);

	// 则：修正后的倾斜叠加目标偏航旋转所得的倾斜向量，与
	// 原始倾斜叠加飞行器当前偏航旋转所得的倾斜向量相同
	EXPECT_TRUE(isEqual((q_sp_yaw * q_tilt_sp_ne).dcm_z(), (q_yaw * q_tilt_sp_ne_before).dcm_z()));
}
