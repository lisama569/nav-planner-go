// Package kinematics 定义差速驱动（differential-drive）机器人的位姿、
// 运动学约束与轨迹校验。
//
// 约束模型：
//   - 线速度 V 限制在 [MinV, MaxV]（默认只前进不倒退，MinV=0）；
//   - 角速度 Omega 限制在 [-MaxOmega, MaxOmega]；原地转向时 V=0；
//   - 曲率 kappa=|Omega/V| 在行进段不得超过 MaxCurvature；
//   - 单段朝向变化（沿最短角方向包裹到 [-pi,pi]）不得超过 MaxDTheta；
//   - 每段必须能用 (V, Omega, DT) 按差速运动学精确积分到下一个位姿，
//     从而保证“位姿连续”且指令与实际运动一致；
//   - DT 必须严格为正。
package kinematics

import (
	"fmt"
	"math"
)

// Pose 为平面位姿：X/Y 为地图坐标，Theta 为朝向角（弧度，逆时针为正）。
type Pose struct {
	X, Y, Theta float64
}

// Twist 为体坐标系速度指令：线速度 V 与角速度 Omega。
type Twist struct {
	V, Omega float64
}

// Constraints 是轨迹必须满足的运动学与速度约束。
type Constraints struct {
	MaxV           float64 // 最大前进线速度（栅格/秒）
	MinV           float64 // 最小线速度（允许倒车时为负）
	MaxOmega       float64 // 最大角速度（弧度/秒）
	MaxCurvature   float64 // 最大曲率 |Omega/V|
	MaxDTheta      float64 // 单段最大转角变化（弧度）
	MaxLateralJerk float64 // 侧向冲击上限（保留项，差速模型不直接产生侧滑）
}

// ViolationCode 标识轨迹违反的约束类别，可用于统计与分类。
type ViolationCode int

const (
	ViolationNone ViolationCode = iota
	ViolationVelocity
	ViolationAngularVelocity
	ViolationCurvature
	ViolationTurnAngle
	ViolationDiscontinuity
	ViolationTiming
)

func (c ViolationCode) String() string {
	switch c {
	case ViolationVelocity:
		return "velocity"
	case ViolationAngularVelocity:
		return "angular-velocity"
	case ViolationCurvature:
		return "curvature"
	case ViolationTurnAngle:
		return "turn-angle"
	case ViolationDiscontinuity:
		return "discontinuity"
	case ViolationTiming:
		return "timing"
	default:
		return "none"
	}
}

// Violation 描述一处约束违反；Index 为段序号（从 0 起，-1 表示整段轨迹级问题）。
type Violation struct {
	Code  ViolationCode
	Index int
	Msg   string
}

func (v Violation) Error() string {
	return fmt.Sprintf("kinematics: %s violation at segment %d: %s", v.Code, v.Index, v.Msg)
}

// Point 是轨迹采样点：该段以 (V, Omega) 持续 DT 秒后到达 Pose。
type Point struct {
	Pose  Pose
	V     float64
	Omega float64
	DT    float64
}

// Trajectory 是一串时间有序的轨迹点（每点描述一个运动段）。
type Trajectory struct {
	Points []Point
}

// 校验/积分使用的数值容差。
const (
	epsV     = 1e-9
	epsOmega = 1e-9
	integTol = 1e-7
)

// DefaultConstraints 返回演示机器人的默认约束：
// 最高 1 栅格/秒、最高 90°/秒、行进曲率上限 1.0（1/栅格）、单段最多转 90°。
func DefaultConstraints() Constraints {
	return Constraints{
		MaxV:           1.0,
		MinV:           0.0,
		MaxOmega:       math.Pi / 2,
		MaxCurvature:   1.0,
		MaxDTheta:      math.Pi / 2,
		MaxLateralJerk: 2.0,
	}
}

// End 返回轨迹末端位姿，空轨迹返回零位姿。
func (t *Trajectory) End() Pose {
	if t == nil || len(t.Points) == 0 {
		return Pose{}
	}
	return t.Points[len(t.Points)-1].Pose
}

// Duration 返回轨迹总时长。
func (t *Trajectory) Duration() float64 {
	var d float64
	if t == nil {
		return 0
	}
	for _, p := range t.Points {
		d += p.DT
	}
	return d
}

// wrapAngle 把角度差包裹到 (-pi, pi]。
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// Integrate 是差速驱动单段精确积分（直线或圆弧），用于校验位姿连续性。
func Integrate(p Pose, v, omega, dt float64) Pose {
	if math.Abs(omega) < epsOmega {
		return Pose{
			X:     p.X + v*math.Cos(p.Theta)*dt,
			Y:     p.Y + v*math.Sin(p.Theta)*dt,
			Theta: p.Theta,
		}
	}
	r := v / omega
	return Pose{
		X:     p.X - r*math.Sin(p.Theta) + r*math.Sin(p.Theta+omega*dt),
		Y:     p.Y + r*math.Cos(p.Theta) - r*math.Cos(p.Theta+omega*dt),
		Theta: p.Theta + omega*dt,
	}
}

// Validate 校验轨迹：
// 首段必须从 start 精确出发（位姿连续）；随后逐段检查时序、速度、角速度、
// 曲率、转角以及“按指令积分是否到达下一点位姿”。返回所有违规（不止第一个）。
func Validate(t *Trajectory, start Pose, c Constraints) []Violation {
	var vs []Violation
	add := func(code ViolationCode, idx int, format string, args ...any) {
		vs = append(vs, Violation{Code: code, Index: idx, Msg: fmt.Sprintf(format, args...)})
	}
	if t == nil || len(t.Points) == 0 {
		add(ViolationTiming, -1, "empty trajectory")
		return vs
	}

	cur := start
	for i, p := range t.Points {
		// 时序
		if p.DT <= 0 {
			add(ViolationTiming, i, "dt=%.6g must be positive", p.DT)
		}
		// 线速度
		if p.V < c.MinV-epsV || p.V > c.MaxV+epsV {
			add(ViolationVelocity, i, "v=%.6g out of [%.6g, %.6g]", p.V, c.MinV, c.MaxV)
		}
		// 角速度
		if math.Abs(p.Omega) > c.MaxOmega+epsOmega {
			add(ViolationAngularVelocity, i, "|omega|=%.6g exceeds %.6g", math.Abs(p.Omega), c.MaxOmega)
		}
		// 曲率（仅行进段约束）
		if math.Abs(p.V) > epsV {
			kappa := math.Abs(p.Omega / p.V)
			if kappa > c.MaxCurvature+1e-9 {
				add(ViolationCurvature, i, "kappa=%.6g exceeds %.6g", kappa, c.MaxCurvature)
			}
		}
		// 转角
		dTh := wrapAngle(p.Pose.Theta - cur.Theta)
		if math.Abs(dTh) > c.MaxDTheta+1e-9 {
			add(ViolationTurnAngle, i, "|dtheta|=%.6g exceeds %.6g", math.Abs(dTh), c.MaxDTheta)
		}
		// 位姿连续性与运动学一致性
		if p.DT > 0 {
			want := Integrate(cur, p.V, p.Omega, p.DT)
			dx := want.X - p.Pose.X
			dy := want.Y - p.Pose.Y
			dw := math.Abs(wrapAngle(want.Theta - p.Pose.Theta))
			if math.Abs(dx) > integTol || math.Abs(dy) > integTol || dw > integTol {
				add(ViolationDiscontinuity, i,
					"integrate (v=%.6g,w=%.6g,dt=%.6g) -> (%.6g,%.6g,%.6g) but point is (%.6g,%.6g,%.6g)",
					p.V, p.Omega, p.DT, want.X, want.Y, want.Theta, p.Pose.X, p.Pose.Y, p.Pose.Theta)
			}
		}
		cur = p.Pose
	}
	return vs
}
