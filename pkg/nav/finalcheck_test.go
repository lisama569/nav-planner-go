package nav

import (
	"errors"
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/planner"
)

// corridorMap: 水平走廊 y=1..2（上下都是墙），从 x=1 到 x=13 无其他通路；
// 走廊足够宽，A* 路径走 y=1。
func corridorMap() *grid.Grid {
	g := grid.New(15, 5)
	for x := 0; x < 15; x++ {
		g.Set(grid.Cell{X: x, Y: 0}, true)
		g.Set(grid.Cell{X: x, Y: 3}, true)
		g.Set(grid.Cell{X: x, Y: 4}, true)
	}
	return g
}

// TestDeliveredTrajectoryFinalCheckGrazing 构造一个“路径单元格全都自由、
// 但按这些格中心连成的整条直线轨迹会擦到圆形障碍（侵入安全裕度）”的场景。
// 旧版只按静态地图走的一次性 Plan 会把这条穿障轨迹当成功；
// 新版终检必须拦住：触发膨胀重规划，走廊被封死则明确判 BLOCKED，
// 且绝不返回状态成功、轨迹却穿障的半成品。
func TestDeliveredTrajectoryFinalCheckGrazing(t *testing.T) {
	g := corridorMap()
	n := newTestNav(g)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: g})
	req := Request{Seq: 1, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 1}}

	// 障碍中心 (3,1.62) r=0.5 margin=0.25：所在格 (3,2) 自由，
	// 但 y=1 的直行轨迹在 x≈2.75 处侵入裕度（探针实测）。
	obs := local.Obstacle{ID: "grazer", X: 3, Y: 1.62, Radius: 0.5, Seq: 2}
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: obs}); err != nil {
		t.Fatal(err)
	}

	r := n.Plan(req)
	logResult(t, "grazing/Plan", req, r)
	if r.Status == StatusPlanned || r.Status == StatusReplanned {
		// 成功类结论的轨迹必须真的不碰障碍（本场景不应走到这里）
		snapG, snapObs := n.Snapshot()
		if hit, col, id := n.lp.FirstCollision(r.Traj, snapG, snapObs); hit {
			t.Fatalf("状态=%s 但终检碰撞 obstacle=%s %s：穿障半成品被下发", r.Status, id, col.Msg)
		}
	}
	if r.Status != StatusUnreachable || !errors.Is(r.Reason, planner.ErrUnreachable) {
		t.Fatalf("走廊被擦碰障碍封死、重搜不可达 => want UNREACHABLE/ErrUnreachable got %s (%v)",
			r.Status, r.Reason)
	}
	if r.Traj != nil || len(r.Path) != 0 {
		t.Fatal("失败结论不得下发半成品 path/trajectory")
	}
	t.Logf("[grazing] 判定依据: 直路终检碰撞 => 膨胀重规划 => 走廊封死重搜不可达 => UNREACHABLE，replans=%d，无穿障轨迹下发",
		r.Replans)

	// 逐步入口结论必须一致
	rs := n.ReplanningStep(req)
	logResult(t, "grazing/ReplanningStep", req, rs)
	if rs.Status != StatusUnreachable || !errors.Is(rs.Reason, planner.ErrUnreachable) {
		t.Fatalf("逐步入口应一致判 UNREACHABLE/ErrUnreachable，got %s/%v", rs.Status, rs.Reason)
	}
}

// TestFinalCheckPassesDetour 终检发现冲突后若存在绕行空间，必须重规划成功，
// 且下发的整条绕行轨迹逐段复核无碰（不是只在局部小段无碰）。
func TestFinalCheckPassesDetour(t *testing.T) {
	g := grid.New(15, 7) // 无静态墙，四周留绕行空间
	n := newTestNav(g)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: g})
	req := Request{Seq: 1, Tick: 1, Start: kinematics.Pose{X: 1, Y: 3}, Goal: grid.Cell{X: 13, Y: 3}}

	// 直线路径 y=3 会擦到障碍，但上方/下方有开阔空间可绕。
	obs := local.Obstacle{ID: "grazer", X: 7, Y: 3.62, Radius: 0.5, Seq: 2}
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: obs}); err != nil {
		t.Fatal(err)
	}
	for _, name := range []string{"Plan", "ReplanningStep"} {
		var r Result
		if name == "Plan" {
			r = n.Plan(req)
		} else {
			r = n.ReplanningStep(req)
		}
		logResult(t, "detour/"+name, req, r)
		if r.Status != StatusReplanned {
			t.Fatalf("%s: 擦碰直路+可绕行 => want REPLANNED got %s (%v)", name, r.Status, r.Reason)
		}
		snapG, snapObs := n.Snapshot()
		if hit, col, id := n.lp.FirstCollision(r.Traj, snapG, snapObs); hit {
			t.Fatalf("%s: 绕行轨迹终检仍碰撞 obstacle=%s %s", name, id, col.Msg)
		}
		// 绕行路径不得经过障碍足迹格 (7,4)
		if contains(r.Path, grid.Cell{X: 7, Y: 4}) {
			t.Fatalf("%s: 绕行路径仍踩障碍足迹 (7,4): %v", name, r.Path)
		}
		t.Logf("[detour/%s] 判定依据: 直路终检碰撞 => 膨胀重规划，整轨迹 %d 段逐段无碰，路径=%v",
			name, trajLen(r.Traj), r.Path)
	}
}

// TestFinalCheckGuardWithStubPlanner 用测试接缝模拟“规划器每次都给出一条
// 格中心路径、但按构造连成的整条轨迹始终擦碰障碍”的漂移输出：
// 即便搜索层声称成功，下发前的整轨迹终检闸门也必须拦住——
// 继续重规划直至耗尽预算，最终判 BLOCKED + ErrTrajectoryCollision，
// 绝不出现“状态 PLANNED、轨迹却穿障”的交付。
func TestFinalCheckGuardWithStubPlanner(t *testing.T) {
	g := grid.New(15, 5)
	n := newTestNav(g)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: g})
	// 桩：无论工作地图怎么膨胀，永远返回沿 y=2 的直线路径。
	straight := make([]grid.Cell, 0, 13)
	for x := 1; x <= 13; x++ {
		straight = append(straight, grid.Cell{X: x, Y: 2})
	}
	n.planPathFn = func(_ *grid.Grid, _, _ grid.Cell) ([]grid.Cell, error) {
		return append([]grid.Cell(nil), straight...), nil
	}
	// 障碍在 (7,2) 格内但偏移到上沿：膨胀后桩仍无视它返回穿障路径。
	obs := local.Obstacle{ID: "drift", X: 7, Y: 2.05, Radius: 0.8, Seq: 2}
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: obs}); err != nil {
		t.Fatal(err)
	}
	req := Request{Seq: 1, Tick: 1, Start: kinematics.Pose{X: 1, Y: 2}, Goal: grid.Cell{X: 13, Y: 2}}

	for _, name := range []string{"Plan", "ReplanningStep"} {
		var r Result
		if name == "Plan" {
			r = n.Plan(req)
		} else {
			r = n.ReplanningStep(req)
		}
		logResult(t, "guard-stub/"+name, req, r)
		if r.Status == StatusPlanned || r.Status == StatusReplanned {
			t.Fatalf("%s: 桩规划器输出穿障轨迹时终检闸门必须拦截，却下发了 %s", name, r.Status)
		}
		if r.Status != StatusBlocked || !errors.Is(r.Reason, ErrTrajectoryCollision) {
			t.Fatalf("%s: want BLOCKED/ErrTrajectoryCollision got %s/%v", name, r.Status, r.Reason)
		}
		if r.Traj != nil || len(r.Path) != 0 {
			t.Fatalf("%s: 闸门拦截后不得残留半成品 path/trajectory", name)
		}
		t.Logf("[guard-stub/%s] 判定依据: 每次重搜的轨迹终检均碰撞 => 耗尽 %d 次重规划 => BLOCKED，无穿障交付",
			name, r.Replans)
	}
}
