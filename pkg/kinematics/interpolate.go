package kinematics

import (
	"errors"
	"math"

	"github.com/lisama569/nav-planner-go/pkg/grid"
)

// 轨迹构造相关错误。
var (
	// ErrShortPath 表示路径少于两个单元格，无法构造运动段。
	ErrShortPath = errors.New("kinematics: path has fewer than two cells")
	// ErrBadTimestep 表示给定的仿真步长非正。
	ErrBadTimestep = errors.New("kinematics: dt must be positive")
	// ErrBadConstraints 表示约束参数无法构造合法轨迹。
	ErrBadConstraints = errors.New("kinematics: constraints must have positive MaxV/MaxOmega/MaxDTheta")
)

// CellWorld 定义栅格中心到世界坐标的映射：单元格 (x,y) 的中心即 (x, y)。
// 全局采用这一简单映射，避免多套坐标转换导致的不一致。
func CellWorld(c grid.Cell) (float64, float64) {
	return float64(c.X), float64(c.Y)
}

// PathToTrajectory 将四邻域栅格路径转换为满足约束的差速驱动轨迹。
//
// 构造方式（保证按构造合法，nav 层仍会调用 Validate 复核）：
//  1. 每条网格边先以“原地分段转向”对齐边的朝向（V=0，|Omega|=MaxOmega，
//     每段转角不超过 MaxDTheta）；
//  2. 再以 MaxV 匀速直行若干段到达下一个单元格中心，段数由 dt 决定，
//     每段时长相等且为正，末端精确落在目标点；
//  3. 首段从传入的 start 精确出发，保证与正在执行的轨迹位姿连续。
func PathToTrajectory(path []grid.Cell, start Pose, c Constraints, dt float64) (*Trajectory, error) {
	if len(path) < 2 {
		return nil, ErrShortPath
	}
	if dt <= 0 {
		return nil, ErrBadTimestep
	}
	if c.MaxV <= 0 || c.MaxOmega <= 0 || c.MaxDTheta <= 0 {
		return nil, ErrBadConstraints
	}

	t := &Trajectory{}
	cur := start
	for i := 0; i+1 < len(path); i++ {
		tx, ty := CellWorld(path[i+1])
		dx, dy := tx-cur.X, ty-cur.Y
		d := math.Hypot(dx, dy)
		if d < 1e-12 {
			continue // 已在目标单元格中心
		}
		heading := math.Atan2(dy, dx)

		// 1) 原地分段转向
		cur = appendRotation(t, cur, heading, c)

		// 2) 匀速直行（Omega=0，自然满足曲率约束）
		n := int(math.Ceil(d / (c.MaxV * dt)))
		if n < 1 {
			n = 1
		}
		segDT := d / (c.MaxV * float64(n))
		for k := 0; k < n; k++ {
			next := Integrate(cur, c.MaxV, 0, segDT)
			t.Points = append(t.Points, Point{Pose: next, V: c.MaxV, Omega: 0, DT: segDT})
			cur = next
		}
	}
	return t, nil
}

// appendRotation 把 cur 原地转到 targetTheta，按 MaxDTheta 分段，返回新位姿。
func appendRotation(t *Trajectory, cur Pose, targetTheta float64, c Constraints) Pose {
	dTh := wrapAngle(targetTheta - cur.Theta)
	if math.Abs(dTh) < 1e-12 {
		return cur
	}
	steps := int(math.Ceil(math.Abs(dTh) / c.MaxDTheta))
	if steps < 1 {
		steps = 1
	}
	segAngle := dTh / float64(steps)
	omega := segAngle * c.MaxOmega / math.Abs(segAngle) // 保持 |omega|=MaxOmega
	segDT := math.Abs(segAngle) / c.MaxOmega
	theta := cur.Theta
	for k := 0; k < steps; k++ {
		theta += segAngle
		next := Pose{X: cur.X, Y: cur.Y, Theta: theta}
		t.Points = append(t.Points, Point{Pose: next, V: 0, Omega: omega, DT: segDT})
	}
	return Pose{X: cur.X, Y: cur.Y, Theta: cur.Theta + dTh}
}
