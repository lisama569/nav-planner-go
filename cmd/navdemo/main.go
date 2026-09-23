// Command navdemo 在本地构造地图与障碍物场景，演示完整导航流程：
// 全局规划 → 动态障碍出现并绕行重规划 → 彻底封堵后不可达 → 非法输入分类 → 确定性重放。
package main

import (
	"errors"
	"fmt"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/nav"
	"github.com/lisama569/nav-planner-go/pkg/planner"
	"github.com/lisama569/nav-planner-go/pkg/replay"
)

func main() {
	fmt.Println("=== navdemo: 本地可复现导航与路径规划 ===")

	// 15x10 地图：x=7 一堵竖墙，留两个缺口：近路 (7,4) 与绕行远路 (7,0)。
	base := buildMap()
	fmt.Printf("地图: %dx%d, 占据单元格数=%d\n", base.Width(), base.Height(), countOccupied(base))

	cons := kinematics.DefaultConstraints()
	lp := local.New(local.Config{
		DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80,
	}, cons)
	newNav := func() *nav.Navigator {
		return nav.New(planner.New(0), lp, base)
	}

	n := newNav()
	must(n.ApplyMap(nav.MapUpdate{Seq: 1, Grid: base}), "load map")

	start := kinematics.Pose{X: 1, Y: 1, Theta: 0}
	goal := grid.Cell{X: 13, Y: 8}

	// 1) 初始全局规划：路径穿过 (7,4) 缺口
	r := n.ReplanningStep(nav.Request{Seq: 100, Tick: 1, Start: start, Goal: goal})
	printResult("① 初始请求", start, goal, r)

	// 2) 动态障碍堵住缺口：局部执行受阻 → 障碍足迹膨胀 → 绕行重规划
	mover := local.Obstacle{ID: "mover-1", X: 7, Y: 4, Radius: 0.7, Seq: 2}
	must(n.ApplyObstacle(nav.ObstacleUpdate{Seq: 2, Obstacle: mover}), "inject mover")
	fmt.Printf("注入动态障碍: id=%s at=(%.1f,%.1f) r=%.1f -> 堵住缺口 (7,4)\n",
		mover.ID, mover.X, mover.Y, mover.Radius)
	r2 := n.ReplanningStep(nav.Request{Seq: 101, Tick: 2, Start: start, Goal: goal})
	printResult("② 遇障后重规划请求", start, goal, r2)

	// 3) 乱序观测必须被丢弃，状态保持一致
	stale := local.Obstacle{ID: "mover-1", X: 7, Y: 99, Radius: 0.7, Seq: 1}
	err := n.ApplyObstacle(nav.ObstacleUpdate{Seq: 1, Obstacle: stale})
	fmt.Printf("③ 乱序更新 (seq=1 < 已应用 2): err=%v | 被丢弃=%v\n\n",
		err, errors.Is(err, nav.ErrStaleUpdate))

	// 4) 再堵死绕行缺口 (7,0)：两个缺口都被封，目标不可达
	sealLow := local.Obstacle{ID: "seal-low", X: 7, Y: 0, Radius: 0.7, Seq: 3}
	must(n.ApplyObstacle(nav.ObstacleUpdate{Seq: 3, Obstacle: sealLow}), "inject seal-low")
	fmt.Printf("注入障碍: id=%s at=(%.1f,%.1f) r=%.1f -> 堵住绕行缺口 (7,0)\n",
		sealLow.ID, sealLow.X, sealLow.Y, sealLow.Radius)
	r3 := n.ReplanningStep(nav.Request{Seq: 102, Tick: 3, Start: start, Goal: goal})
	printResult("④ 两个缺口均被封死", start, goal, r3)

	// 5) 非法输入：目标越界 / 目标占据 / 地图缺失，原因必须可区分
	bad1 := n.Plan(nav.Request{Seq: 103, Tick: 4, Start: start, Goal: grid.Cell{X: 99, Y: 99}})
	printResult("⑤a 目标越界", start, grid.Cell{X: 99, Y: 99}, bad1)
	bad2 := n.Plan(nav.Request{Seq: 104, Tick: 5, Start: start, Goal: grid.Cell{X: 7, Y: 1}})
	printResult("⑤b 目标在墙身(7,1)（占据）", start, grid.Cell{X: 7, Y: 1}, bad2)
	emptyNav := nav.New(planner.New(0), lp, nil)
	bad3 := emptyNav.Plan(nav.Request{Seq: 105, Tick: 6, Start: start, Goal: goal})
	printResult("⑤c 地图缺失", start, goal, bad3)

	// 6) 确定性重放：同一事件序列在全新编排器上逐条比对必须一致
	fmt.Println("--- 确定性重放校验 ---")
	rec := replay.NewRecorder(newNav())
	rec.Apply(replay.Event{Tick: 1, Kind: replay.EventMap, Map: base})
	rec.Apply(replay.Event{Tick: 2, Kind: replay.EventObstacle, Obstacle: mover})
	rec.Apply(replay.Event{Tick: 100, Kind: replay.EventRequest, Start: start, Goal: goal})
	ok, err := replay.Replay(rec.Log(), newNav())
	fmt.Printf("重放条目=%d，结论逐条一致=%v，err=%v\n", len(rec.Log().Entries), ok, err)
}

// buildMap 构造 15x10 地图：x=7 的竖墙在 (7,4) 与 (7,0) 各留一个缺口。
func buildMap() *grid.Grid {
	g := grid.New(15, 10)
	for y := 0; y < g.Height(); y++ {
		if y == 4 || y == 0 {
			continue
		}
		g.Set(grid.Cell{X: 7, Y: y}, true)
	}
	return g
}

func countOccupied(g *grid.Grid) int {
	n := 0
	for x := 0; x < g.Width(); x++ {
		for y := 0; y < g.Height(); y++ {
			if g.Occupied(grid.Cell{X: x, Y: y}) {
				n++
			}
		}
	}
	return n
}

func must(err error, what string) {
	if err != nil {
		panic(fmt.Sprintf("%s failed: %v", what, err))
	}
}

func printResult(tag string, start kinematics.Pose, goal grid.Cell, r nav.Result) {
	fmt.Printf("[%s]\n", tag)
	fmt.Printf("  输入: start=(%.2f,%.2f,th=%.2f) goal=(%d,%d)\n",
		start.X, start.Y, start.Theta, goal.X, goal.Y)
	fmt.Printf("  结果: status=%s reason=%v replans=%d expansions=%d\n",
		r.Status, r.Reason, r.Replans, r.Expansions)
	if len(r.Path) > 0 {
		fmt.Printf("  路径(%d): %v\n", len(r.Path), r.Path)
	}
	if r.Traj != nil {
		fmt.Printf("  轨迹: %d 段, 时长=%.2fs, 末端=(%.3f,%.3f,th=%.3f)\n",
			len(r.Traj.Points), r.Traj.Duration(),
			r.Traj.End().X, r.Traj.End().Y, r.Traj.End().Theta)
	}
	verdict := "拒绝/失败（不可下发）"
	if r.Status == nav.StatusPlanned || r.Status == nav.StatusReplanned {
		verdict = "有效轨迹（可下发）"
	}
	fmt.Printf("  判定依据: %s\n\n", verdict)
}
