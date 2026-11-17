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
 * @file AttitudeControl.hpp
 *
 * 基于四元数的姿态控制器。
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * 本实现的四元数姿态控制算法参考以下论文：
 * Nonlinear Quadrocopter Attitude Control（非线性四旋翼姿态控制，2013）
 * 作者：Dario Brescianini、Markus Hehn 和 Raffaello D'Andrea
 * 苏黎世联邦理工学院动态系统与控制研究所（IDSC）
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * 设置姿态控制的比例增益
	 * @param proportional_gain 包含横滚、俯仰、偏航增益的三维向量
	 * @param yaw_weight [0,1] 范围内的权重，用于降低偏航相对于横滚和俯仰的优先程度
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * 设置输出目标角速度的硬限幅
	 * @param rate_limit [rad/s] 包含横滚、俯仰、偏航角速度限制的三维向量
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * 设置新的目标姿态，替换此前跟踪的目标
	 * @param qd 飞行器的目标姿态
	 * @param yawspeed_setpoint [rad/s] 世界系中的偏航前馈角速度
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * 使用一个增量旋转调整最近保存的目标姿态
	 * 可在姿态估计参考发生变化（例如航向改变）时使用，以避免目标突变。
	 * @param q_delta 要施加的增量旋转
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * 执行一次控制循环计算
	 * @param q 当前飞行器姿态估计的单位四元数
	 * @return [rad/s] 机体系中的三维目标角速度向量，交由角速度控制器跟踪
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

private:
	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< 偏航权重 [0,1]，用于降低偏航相对于横滚和俯仰的优先程度

	matrix::Quatf _attitude_setpoint_q; ///< 最近保存的目标姿态，例如来自位置控制器的目标
	float _yawspeed_setpoint{0.f}; ///< 最近保存的偏航角速度前馈目标值
};
