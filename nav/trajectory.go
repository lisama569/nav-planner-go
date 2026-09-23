package nav

import (
	"math"
	"time"
)

// TrajectoryBuilder 依据全局路径生成满足运动学约束的离散轨迹。
//
// 运动模型（差速/独轮车，固定离散步长 DT）：
//
//	x'     = x + v*cos(theta)*DT
//	y'     = y + v*sin(theta)*DT
//	theta' = theta + omega*DT
//
// 约定：点 i 记录的 (v,omega) 是作用于区间 [i-1,i] 的指令速度，
// 因此相邻点位姿必须能用该指令由上一点积分复现（见 Validate）。
//
// 生成策略：“先原地转向对齐段方向 → 梯形速度直线推进 → 末端对齐目标航向”。
// 平移段曲率恒为 0，天然满足 MaxCurvature。每个平移段的速度曲线按以下
// 方式严格构造，保证总位移精确等于段长且从 0 起步、到节点归 0：
//
//  1. 求最少步数 N，使双向加速度受限的最大速度序列
//     w_i = min(Vmax, i*dv, (N+1-i)*dv)（dv=MaxAccel*DT）总行程 >= 段长；
//  2. 等比缩放 f = 段长 / Σ(w_i*DT)，取 v_i = f*w_i。
//     缩放后总位移精确等于段长，且任意相邻速度差 <= dv，自动满足加速度约束。
type TrajectoryBuilder struct {
	cfg Config
}

// NewTrajectoryBuilder 创建轨迹生成器。
func NewTrajectoryBuilder(cfg Config) *TrajectoryBuilder { return &TrajectoryBuilder{cfg: cfg} }

const (
	// poseEps 是位姿连续性校验的绝对容差。
	poseEps = 1e-6
	// tinyDist 小于该距离的平移段视为零长，直接跳过。
	tinyDist = 1e-9
)

// Build 从 start 位姿出发沿 path 生成轨迹。
//
// 返回的 Trajectory 永远非 nil：
//   - 正常生成且通过 Validate：Status=TrajValid；
//   - 生成过程超过 deadline：Status=TrajTimeout（只含已生成部分，
//     不到达目标，绝不允许下发）；
//   - 生成结果自检违反约束：Status=TrajConstraintViolation 并附原因。
func (b *TrajectoryBuilder) Build(start, goal Pose, path *Path, deadline *time.Time, clk Clock) *Trajectory {
	if clk == nil {
		clk = WallClock{}
	}
	expired := func() bool { return deadline != nil && !clk.Now().Before(*deadline) }

	if path == nil || len(path.Nodes) == 0 {
		return &Trajectory{Status: TrajConstraintViolation,
			Reason: "cannot build trajectory from empty path"}
	}
	if isNaNpose(start) || isNaNpose(goal) {
		return &Trajectory{Status: TrajConstraintViolation,
			Reason: "start or goal pose contains NaN"}
	}

	dt := b.cfg.DT.Seconds()
	lim := b.cfg.Limits
	dv := lim.MaxAccel * dt
	// 单步最大角速度同时受角速度上限与转向速率限制。
	maxOmega := math.Min(lim.MaxAngularVel, lim.MaxSteerRate/dt)

	pts := []TrajPoint{{Pose: start, Time: 0, LinearVel: 0, AngularVel: 0}}
	t := 0.0
	x, y, theta := start.X, start.Y, start.Theta

	// appendPoint 追加一个积分点。
	appendPoint := func(nx, ny, ntheta, v, omega float64) {
		t += dt
		pts = append(pts, TrajPoint{
			Pose:       Pose{X: nx, Y: ny, Theta: ntheta},
			Time:       durSec(t),
			LinearVel:  v,
			AngularVel: omega,
		})
	}

	// turnTo 原地转向到目标航向（v=0），末步不越过目标航向。
	turnTo := func(targetTheta float64) *Trajectory {
		for {
			if expired() {
				return &Trajectory{Points: pts, Status: TrajTimeout,
					Reason: "trajectory generation timed out during in-place turn"}
			}
			dh := wrapAngle(targetTheta - theta)
			if math.Abs(dh) <= poseEps {
				return nil
			}
			// 先按单步航向变化上限限制角速度（同时也满足 MaxStepTheta）。
			omega := clamp(math.Abs(dh)/dt, 0, math.Min(maxOmega, lim.MaxStepTheta/dt))
			if dh < 0 {
				omega = -omega
			}
			step := omega * dt
			if math.Abs(step) > math.Abs(dh) {
				omega = dh / dt
				step = dh
			}
			theta += step
			appendPoint(x, y, theta, 0, omega)
		}
	}

	// 依次追随路径节点（首个与当前位置重合的节点跳过）。
	for i := range path.Nodes {
		nx, ny := path.Nodes[i].X, path.Nodes[i].Y
		dx, dy := nx-x, ny-y
		dist := math.Hypot(dx, dy)
		if dist <= tinyDist {
			continue
		}
		if tr := turnTo(math.Atan2(dy, dx)); tr != nil {
			return tr
		}

		// 求最少步数 N 与最大速度序列 w_i。
		var N int
		var sumW float64
		for {
			if expired() {
				return &Trajectory{Points: pts, Status: TrajTimeout,
					Reason: "trajectory generation timed out while planning segment to " +
						path.Nodes[i].C.String()}
			}
			N++
			sumW = 0
			for k := 1; k <= N; k++ {
				sumW += math.Min(lim.MaxLinearVel,
					math.Min(float64(k)*dv, float64(N+1-k)*dv))
			}
			if sumW*dt >= dist {
				break
			}
			if N > 1_000_000 {
				return &Trajectory{Points: pts, Status: TrajConstraintViolation,
					Reason: "segment too long to discretize within step limit"}
			}
		}
		f := dist / (sumW * dt)

		// 发射 N 个平移指令，精确走完 dist。
		for k := 1; k <= N; k++ {
			if expired() {
				return &Trajectory{Points: pts, Status: TrajTimeout,
					Reason: "trajectory generation timed out while translating to " +
						path.Nodes[i].C.String()}
			}
			w := math.Min(lim.MaxLinearVel,
				math.Min(float64(k)*dv, float64(N+1-k)*dv))
			v := f * w
			step := v * dt
			x += math.Cos(theta) * step
			y += math.Sin(theta) * step
			appendPoint(x, y, theta, v, 0)
		}
	}

	// 末端对齐目标航向，并追加零指令点表示真正停稳。
	if tr := turnTo(goal.Theta); tr != nil {
		return tr
	}
	t += dt
	pts = append(pts, TrajPoint{
		Pose: Pose{X: x, Y: y, Theta: theta}, Time: durSec(t),
		LinearVel: 0, AngularVel: 0,
	})

	traj := &Trajectory{Points: pts}
	if reason := b.Validate(traj); reason != "" {
		traj.Status = TrajConstraintViolation
		traj.Reason = reason
		return traj
	}
	traj.Status = TrajValid
	return traj
}

// Validate 校验轨迹连续性与全部运动学/速度约束，返回第一条违规的描述。
// 校验依据：
//  1. 至少 2 个点且不含 NaN；
//  2. 相邻点位姿必须严格等于“上一点位姿 + 当前点指令(v,omega)积分 DT”；
//  3. 速度不越限、相邻指令加速度/转向速率不越限、单步航向变化不越限；
//  4. 平移过程中曲率 |omega/v| 不超过 MaxCurvature。
func (b *TrajectoryBuilder) Validate(t *Trajectory) string {
	if t == nil {
		return "trajectory is nil"
	}
	if len(t.Points) < 2 {
		return "trajectory has fewer than 2 points"
	}
	lim := b.cfg.Limits
	dt := b.cfg.DT.Seconds()
	stepOmegaMax := math.Min(lim.MaxAngularVel, lim.MaxSteerRate/dt)
	for i, pt := range t.Points {
		if isNaNpose(pt.Pose) || math.IsNaN(pt.LinearVel) || math.IsNaN(pt.AngularVel) {
			return indexReason(i, "pose or velocity contains NaN")
		}
		if pt.LinearVel > lim.MaxLinearVel+poseEps || pt.LinearVel < lim.MinLinearVel-poseEps {
			return indexReason(i, "linear velocity %v exceeds [%v,%v]",
				pt.LinearVel, lim.MinLinearVel, lim.MaxLinearVel)
		}
		if math.Abs(pt.AngularVel) > stepOmegaMax+poseEps {
			return indexReason(i, "angular velocity %v exceeds max %v (steer-rate limited)",
				pt.AngularVel, stepOmegaMax)
		}
		if math.Abs(pt.LinearVel) > poseEps && lim.MaxCurvature > 0 && math.Abs(pt.AngularVel) > poseEps {
			kappa := math.Abs(pt.AngularVel / pt.LinearVel)
			if kappa > lim.MaxCurvature+poseEps {
				return indexReason(i, "curvature %v exceeds max %v", kappa, lim.MaxCurvature)
			}
		}
		if i == 0 {
			if pt.LinearVel != 0 || pt.AngularVel != 0 {
				return indexReason(i, "first point must carry zero command (initial state)")
			}
			continue
		}
		prev := t.Points[i-1]
		stepDt := pt.Time.Seconds() - prev.Time.Seconds()
		if math.Abs(stepDt-dt) > poseEps {
			return indexReason(i, "time step %v != configured DT %v", stepDt, dt)
		}
		ex := prev.Pose.X + pt.LinearVel*math.Cos(prev.Pose.Theta)*dt
		ey := prev.Pose.Y + pt.LinearVel*math.Sin(prev.Pose.Theta)*dt
		eth := prev.Pose.Theta + pt.AngularVel*dt
		if math.Abs(ex-pt.Pose.X) > poseEps || math.Abs(ey-pt.Pose.Y) > poseEps ||
			math.Abs(wrapAngle(eth-pt.Pose.Theta)) > poseEps {
			return indexReason(i,
				"pose discontinuity: expected=(%.9f,%.9f,%.9f) got=(%.9f,%.9f,%.9f)",
				ex, ey, eth, pt.Pose.X, pt.Pose.Y, pt.Pose.Theta)
		}
		if math.Abs(pt.LinearVel-prev.LinearVel) > lim.MaxAccel*dt+poseEps {
			return indexReason(i, "linear acceleration %v exceeds MaxAccel %v",
				math.Abs(pt.LinearVel-prev.LinearVel)/dt, lim.MaxAccel)
		}
		if dTheta := math.Abs(pt.AngularVel * dt); dTheta > lim.MaxStepTheta+poseEps {
			return indexReason(i, "heading step %v exceeds MaxStepTheta %v", dTheta, lim.MaxStepTheta)
		}
	}
	return ""
}

// wrapAngle 将角度归一化到 (-pi, pi]。
func wrapAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func durSec(s float64) time.Duration { return time.Duration(s * float64(time.Second)) }

func indexReason(i int, format string, args ...any) string {
	return NewError(FailConstraint, "traj point #%d: "+format,
		append([]any{i}, args...)...).Msg
}
