package nav

import (
	"errors"
	"testing"
	"time"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
	"github.com/lisama569/nav-planner-go/pkg/local"
	"github.com/lisama569/nav-planner-go/pkg/planner"
)

// newConsistencyNav 构造测试编排器并返回局部规划器（用于独立的碰撞复核）。
func newConsistencyNav(g *grid.Grid) (*Navigator, *local.Planner) {
	lp := local.New(local.Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}, kinematics.DefaultConstraints())
	return New(planner.New(0), lp, g), lp
}

// assertDeliveredTrajSafe 是“已规划/已重规划结论绝不下发穿障轨迹”的统一判定依据。
func assertDeliveredTrajSafe(t *testing.T, name string, lp *local.Planner, g *grid.Grid, obs []local.Obstacle, r Result) {
	t.Helper()
	if r.Status != StatusPlanned && r.Status != StatusReplanned {
		if r.Traj != nil || len(r.Path) != 0 {
			t.Fatalf("[%s] 失败结论 %s 不得携带路径/轨迹", name, r.Status)
		}
		return
	}
	if r.Traj == nil {
		t.Fatalf("[%s] 成功结论 %s 缺少轨迹", name, r.Status)
	}
	if lp.Collides(r.Traj, g, obs) {
		t.Fatalf("[%s] 状态=%s 但交付轨迹穿过已知动态障碍（终检失守）", name, r.Status)
	}
	t.Logf("[%s] 判定依据: status=%s 且交付轨迹对当前 %d 个已知障碍终检通过", name, r.Status, len(obs))
}

// TestEntryConsistency 验证同一世界状态下，一次性入口 Plan 与逐步推进入口
// ReplanningStep 的结论一致且可互相解释：都把已接受的动态障碍算进世界观。
func TestEntryConsistency(t *testing.T) {
	base := twoGapMap()
	req := Request{Seq: 100, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}

	n, lp := newConsistencyNav(base)
	if err := n.ApplyMap(MapUpdate{Seq: 1, Grid: base}); err != nil {
		t.Fatal(err)
	}
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: local.Obstacle{
		ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 2}}); err != nil {
		t.Fatal(err)
	}
	_, obs := n.Snapshot()
	t.Logf("[consistency] 输入状态: 地图=双缺口墙, 已知障碍=%v, start=(1,1) goal=(13,8)", obs)

	// 场景一：障碍堵住近缺口 —— 两个入口都必须绕行远缺口，结论与路线一致。
	rp := n.Plan(req)
	logResult(t, "plan/one-shot", req, rp)
	rs := n.ReplanningStep(req)
	logResult(t, "replanning-step", req, rs)
	if rp.Status != rs.Status {
		t.Fatalf("入口结论不一致: Plan=%s ReplanningStep=%s", rp.Status, rs.Status)
	}
	if rp.Status != StatusReplanned {
		t.Fatalf("两入口都应绕行成功(REPLANNED)，got %s", rp.Status)
	}
	if len(rp.Path) != len(rs.Path) {
		t.Fatalf("两入口路径长度不一致: %d vs %d", len(rp.Path), len(rs.Path))
	}
	for i := range rp.Path {
		if rp.Path[i] != rs.Path[i] {
			t.Fatalf("两入口路径在 %d 处分叉: %v vs %v", i, rp.Path[i], rs.Path[i])
		}
	}
	if contains(rp.Path, grid.Cell{X: 7, Y: 4}) {
		t.Fatalf("Plan 的路线穿过已知障碍堵住的缺口 (7,4): %v", rp.Path)
	}
	assertDeliveredTrajSafe(t, "plan/one-shot", lp, base, obs, rp)
	assertDeliveredTrajSafe(t, "replanning-step", lp, base, obs, rs)

	// 场景二：两个缺口都被封死 —— 两个入口都必须判 UNREACHABLE，原因可互相解释。
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 3, Obstacle: local.Obstacle{
		ID: "s", X: 7, Y: 0, Radius: 0.7, Seq: 3}}); err != nil {
		t.Fatal(err)
	}
	_, obs2 := n.Snapshot()
	t.Logf("[consistency] 输入状态: 两个缺口均被动态障碍封死, 已知障碍=%v", obs2)
	rp2 := n.Plan(req)
	logResult(t, "plan/sealed", req, rp2)
	rs2 := n.ReplanningStep(req)
	logResult(t, "replanning-step/sealed", req, rs2)
	if rp2.Status != StatusUnreachable || rs2.Status != StatusUnreachable {
		t.Fatalf("封死后两入口都应 UNREACHABLE: Plan=%s ReplanningStep=%s", rp2.Status, rs2.Status)
	}
	if !errors.Is(rp2.Reason, planner.ErrUnreachable) || !errors.Is(rs2.Reason, planner.ErrUnreachable) {
		t.Fatalf("两入口失败原因应可互相解释(ErrUnreachable): %v / %v", rp2.Reason, rs2.Reason)
	}
	assertDeliveredTrajSafe(t, "plan/sealed", lp, base, obs2, rp2)
	assertDeliveredTrajSafe(t, "replanning-step/sealed", lp, base, obs2, rs2)

	// 场景三：非法输入 —— 两个入口分类一致。
	bad := Request{Start: req.Start, Goal: grid.Cell{X: 99, Y: 99}}
	bp := n.Plan(bad)
	bs := n.ReplanningStep(bad)
	logResult(t, "plan/bad-goal", bad, bp)
	logResult(t, "replanning-step/bad-goal", bad, bs)
	if bp.Status != bs.Status || bp.Status != StatusRejected {
		t.Fatalf("非法输入两入口分类应一致且为 REJECTED: %s vs %s", bp.Status, bs.Status)
	}
}

// TestFinalTrajectoryRecheck 构造“栅格路径合法、但连续轨迹擦碰障碍”的场景：
// 障碍悬在两个自由格连线的中点上方，A* 路径不穿障碍足迹，交付轨迹却会撞上。
// 终检必须发现它并触发重规划，绝不下发撞障轨迹。
func TestFinalTrajectoryRecheck(t *testing.T) {
	g := grid.New(12, 12)
	n, lp := newConsistencyNav(g)
	if err := n.ApplyMap(MapUpdate{Seq: 1, Grid: g}); err != nil {
		t.Fatal(err)
	}
	// 障碍中心 (4.5,5.9)：格心 (4,5)/(5,5) 距其 1.029 > r+margin=0.95（足迹外），
	// 但连线中点 (4.5,5) 距其 0.9 < 0.95 —— 直线轨迹会撞。
	obs := local.Obstacle{ID: "dip", X: 4.5, Y: 5.9, Radius: 0.7, Seq: 2}
	if err := n.ApplyObstacle(ObstacleUpdate{Seq: 2, Obstacle: obs}); err != nil {
		t.Fatal(err)
	}
	req := Request{Seq: 100, Tick: 1, Start: kinematics.Pose{X: 1, Y: 5}, Goal: grid.Cell{X: 9, Y: 5}}

	// 先证明这确实是个陷阱： naive 直线轨迹会撞上该障碍。
	straight := make([]grid.Cell, 0, 9)
	for x := 1; x <= 9; x++ {
		straight = append(straight, grid.Cell{X: x, Y: 5})
	}
	naive, err := kinematics.PathToTrajectory(straight, req.Start, lp.Constraints(), lp.DT())
	if err != nil {
		t.Fatal(err)
	}
	if !lp.Collides(naive, g, []local.Obstacle{obs}) {
		t.Fatal("前置假设失效：直线轨迹应当撞上 (4.5,5.9) 处的障碍")
	}
	t.Logf("[final-check] 陷阱确认: 栅格路径 y=5 全部自由，但直线轨迹与障碍 r+margin=0.95 相撞")

	r := n.Plan(req)
	logResult(t, "final-check/plan", req, r)
	assertDeliveredTrajSafe(t, "final-check/plan", lp, g, []local.Obstacle{obs}, r)
	if r.Status != StatusReplanned {
		t.Fatalf("终检应触发重规划后绕行成功(REPLANNED)，got %s (%v)", r.Status, r.Reason)
	}

	r2 := n.ReplanningStep(req)
	logResult(t, "final-check/replanning-step", req, r2)
	assertDeliveredTrajSafe(t, "final-check/replanning-step", lp, g, []local.Obstacle{obs}, r2)
	if r2.Status != StatusReplanned {
		t.Fatalf("ReplanningStep 同样应绕行成功(REPLANNED)，got %s (%v)", r2.Status, r2.Reason)
	}
}

// TestPlanTimeoutClassified 验证可配置墙钟兜底：超时必须是可区分的
// “超时/未完成”类结论且不下发半成品；默认（禁用）保持完全确定性。
func TestPlanTimeoutClassified(t *testing.T) {
	base := twoGapMap()
	req := Request{Seq: 100, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}

	// 1) 极小兜底时限：两个入口都必须给出 PLAN_LIMIT + ErrPlanTimeout，且无路径/轨迹。
	n, _ := newConsistencyNav(base)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: base})
	n.SetOptions(Options{PlanTimeout: time.Nanosecond})
	rp := n.Plan(req)
	logResult(t, "timeout/plan", req, rp)
	rs := n.ReplanningStep(req)
	logResult(t, "timeout/replanning-step", req, rs)
	for name, r := range map[string]Result{"Plan": rp, "ReplanningStep": rs} {
		if r.Status != StatusPlanLimit || !errors.Is(r.Reason, ErrPlanTimeout) {
			t.Fatalf("%s: 超时须分类为 PLAN_LIMIT/ErrPlanTimeout，got %s/%v", name, r.Status, r.Reason)
		}
		if r.Traj != nil || len(r.Path) != 0 {
			t.Fatalf("%s: 超时结论不得下发半成品路径/轨迹", name)
		}
	}
	t.Log("[timeout] 判定依据: status=PLAN_LIMIT 且 errors.Is(reason, ErrPlanTimeout)，路径/轨迹为空")

	// 2) 恢复默认（禁用兜底）：同输入必须回到确定性成功结论。
	n.SetOptions(Options{})
	r1 := n.Plan(req)
	logResult(t, "timeout/disabled", req, r1)
	if r1.Status != StatusPlanned {
		t.Fatalf("禁用兜底后应正常 PLANNED，got %s (%v)", r1.Status, r1.Reason)
	}

	// 3) 兜底时限足够大时不得改变正常结论（与禁用模式逐格一致）。
	n.SetOptions(Options{PlanTimeout: time.Hour})
	r2 := n.Plan(req)
	logResult(t, "timeout/generous", req, r2)
	if r2.Status != r1.Status || len(r2.Path) != len(r1.Path) {
		t.Fatalf("充裕时限不应改变结论: %s/%d vs %s/%d", r2.Status, len(r2.Path), r1.Status, len(r1.Path))
	}
	for i := range r1.Path {
		if r1.Path[i] != r2.Path[i] {
			t.Fatalf("充裕时限下路径分叉 at %d: %v vs %v", i, r1.Path[i], r2.Path[i])
		}
	}
}

// TestTimeoutDoesNotCorruptState 确认超时后编排器状态未被污染，后续请求正常。
func TestTimeoutDoesNotCorruptState(t *testing.T) {
	base := twoGapMap()
	n, _ := newConsistencyNav(base)
	_ = n.ApplyMap(MapUpdate{Seq: 1, Grid: base})
	req := Request{Seq: 1, Tick: 1, Start: kinematics.Pose{X: 1, Y: 1}, Goal: grid.Cell{X: 13, Y: 8}}

	n.SetOptions(Options{PlanTimeout: time.Nanosecond})
	if r := n.ReplanningStep(req); r.Status != StatusPlanLimit {
		t.Fatalf("want PLAN_LIMIT got %s", r.Status)
	}
	n.SetOptions(Options{})
	r := n.ReplanningStep(req)
	logResult(t, "timeout/recovery", req, r)
	if r.Status != StatusPlanned || r.Traj == nil {
		t.Fatalf("超时后恢复确定性模式应正常规划，got %s (%v)", r.Status, r.Reason)
	}
}
