/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// 补偿偏航权重对输出幅值缩放的影响
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// 忽略机体偏航，计算约化目标姿态，以优先满足横滚和俯仰控制
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// 在机体方向与期望推力方向完全相反这一极少见的边界情况下，
		// 完整姿态控制本身也不会生成偏航输入，而是直接通过横滚与俯仰的组合
		// 达到正确的目标偏航。因此，即使忽略这一边界情况，系统仍然安全且稳定。
		qd_red = qd;

	} else {
		// 将当前推力向量到目标推力向量的旋转，转换为世界系中的约化目标姿态。
		// 倾斜误差四元数由世界系中表达的两个 Z 向量得到，因此这里采用右乘。
		qd_red *= q;
	}

	// 根据完整目标姿态的关系 qd = qd_red * qd_dyaw，提取偏航增量分量。
	// 按定义，偏航增量四元数的形式为 (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// 限制输入范围，避免数值误差使输入超出 acosf 和 asinf 的定义域
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// 按权重缩放偏航增量角，再重新组合目标姿态
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// 四元数姿态控制律：qe 表示从 q 到 qd 的旋转
	const Quatf qe = q.inversed() * qd;

	// 使用 sin(alpha/2) 缩放的旋转轴表示姿态误差（参见轴角形式的四元数定义）
	// 同时处理互为相反数的单位四元数表示同一姿态所带来的二义性
	const Vector3f eq = 2.f * qe.canonical().imag();

	// 计算目标角速度
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// 对目标偏航角速度进行前馈。
	// yawspeed_setpoint 表示绕世界系 z 轴旋转的前馈指令，
	// 但需要在机体系中施加它（因为 _rates_sp 是在机体系中表达的）。
	// 因此，取 R.transposed（等价于 q.inversed）的最后一列，得到在机体系中表达的世界系 z 轴，
	// 再将其乘以目标偏航角速度（yawspeed_setpoint）。
	// 所得向量是在机体系中表达的、绕世界系 z 轴旋转的角速度指令，
	// 因而可以将其加到目标角速度上。
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// 对各轴目标角速度进行限幅
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
