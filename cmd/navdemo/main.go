// Command navdemo 在本地确定性地演示一次完整导航流程：
// 初始规划 -> 动态障碍侵入触发重规划 -> 障碍合围使目标不可达 ->
// 移除障碍恢复可达。所有结论均带可区分的失败分类与原因。
package main

import (
	"fmt"
	"math"
	"time"

	nav "github.com/lisama569/nav-planner-go/nav"
)

func main() {
	cfg := nav.Config{
		Grid: nav.GridConfig{Rows: 20, Cols: 20, Resolution: 0.5},
		Limits: nav.Limits{
			MaxLinearVel: 1.0, MinLinearVel: 0, MaxAngularVel: math.Pi,
			MaxCurvature: 2.0, MaxAccel: 2.0, MaxSteerRate: math.Pi,
			MaxStepTheta: math.Pi / 2,
		},
		DT: 100 * time.Millisecond, PlanTimeout: time.Second,
		GoalTolerance: 0.5, LocalLookahead: 2.0,
	}

	// FakeClock 保证同一输入序列可逐字节复现。
	clk := nav.NewFakeClock(time.Date(2026, 9, 23, 0, 0, 0, 0, time.UTC))
	n, err := nav.NewNavigator(cfg, clk)
	must(err)
	must(n.SetMap(nav.NewGridMap(cfg.Grid), nil))

	start := nav.Pose{X: 0.5, Y: 0.5}
	goal := nav.Pose{X: 8.0, Y: 8.0}

	report("initial plan", n.Plan(start, goal))

	// 1) 移动障碍落到当前轨迹走廊上 -> 必须重规划。
	ok, why := n.ApplyObstacle(nav.Obstacle{ID: "cart", Seq: 1,
		X: 1.5, Y: 1.5, HalfX: 0.24, HalfY: 0.24})
	fmt.Printf("apply cart@seq1: accepted=%v %s\n", ok, why)
	report("replan around cart", n.Plan(start, goal))

	// 2) 补发到货的旧消息 seq=0 -> 必须被丢弃，状态不变。
	ok, why = n.ApplyObstacle(nav.Obstacle{ID: "cart", Seq: 0,
		X: 9, Y: 9, HalfX: 0.24, HalfY: 0.24})
	fmt.Printf("apply stale cart@seq0: accepted=%v (%s)\n", ok, why)

	// 3) 一排围墙把目标封死 -> 目标不可达，必须被明确拒绝。
	for c := 0; c < 20; c++ {
		ok, why = n.ApplyObstacle(nav.Obstacle{
			ID: fmt.Sprintf("wall-%d", c), Seq: 1,
			X: 0.25 + 0.5*float64(c), Y: 4.0, HalfX: 0.25, HalfY: 0.25,
		})
		if !ok {
			panic(why)
		}
	}
	report("goal sealed off", n.Plan(nav.Pose{X: 4.5, Y: 3.5}, goal))

	// 4) 围墙与小车全部移除 -> 恢复可达。
	for c := 0; c < 20; c++ {
		n.ApplyObstacle(nav.Obstacle{ID: fmt.Sprintf("wall-%d", c), Seq: 2, Removed: true})
	}
	n.ApplyObstacle(nav.Obstacle{ID: "cart", Seq: 2, Removed: true})
	report("after removal", n.Plan(start, goal))

	s := n.State()
	fmt.Printf("final snapshot: hasMap=%v worldSeq=%d liveObstacles=%d\n",
		s.HasMap, s.Seq, s.DynCount)
}

func report(tag string, r *nav.PlanResult) {
	if r.Err != nil {
		fmt.Printf("[%s] REJECTED code=%s reason=%s\n", tag, r.Err.Code, r.Err.Msg)
		return
	}
	fmt.Printf("[%s] OK replanned=%v pathNodes=%d trajPoints=%d seq=%d",
		tag, r.Replanned, len(r.Path.Nodes), len(r.Traj.Points), r.Seq)
	if r.Replanned {
		fmt.Printf("\n        replanReason=%s", r.ReplanReason)
	}
	fmt.Println()
}

func must(err error) {
	if err != nil {
		panic(err)
	}
}
