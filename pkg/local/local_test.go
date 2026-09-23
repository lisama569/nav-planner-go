package local

import (
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
	"github.com/lisama569/nav-planner-go/pkg/kinematics"
)

func testCfg() Config {
	return Config{DT: 0.25, Horizon: 6, VSamples: 3, OmegaSamples: 7,
		GoalTolerance: 0.4, ObstacleMargin: 0.25, MaxIters: 80}
}

// rollout 在测试里重复 nav 的局部推进，用于直接验证 DWA 行为。
func rollout(t *testing.T, p *Planner, start kinematics.Pose, path []grid.Cell, g *grid.Grid, obs []Obstacle) (kinematics.Pose, bool) {
	cur := start
	goal := path[len(path)-1]
	for it := 0; it < p.MaxIters(); it++ {
		if p.Arrived(cur, goal) {
			t.Logf("[rollout] 第%d次迭代到达 %v", it, cur)
			return cur, true
		}
		traj, blocked, err := p.Step(cur, path, g, obs)
		if err != nil {
			t.Fatalf("step err: %v", err)
		}
		if blocked || p.Collides(traj, g, obs) {
			t.Logf("[rollout] 第%d次迭代堵死/碰撞 at=%v", it, cur)
			return cur, false
		}
		cur = traj.End()
	}
	t.Logf("[rollout] 超过最大迭代，停留=%v", cur)
	return cur, false
}

func twoGapMap() *grid.Grid {
	g := grid.New(15, 10)
	for y := 0; y < 10; y++ {
		if y == 4 || y == 0 {
			continue
		}
		g.Set(grid.Cell{X: 7, Y: y}, true)
	}
	return g
}

func TestLocalReachGoalNoObstacle(t *testing.T) {
	g := twoGapMap()
	lp := New(testCfg(), kinematics.DefaultConstraints())
	path := []grid.Cell{{X: 1, Y: 1}, {X: 1, Y: 2}, {X: 1, Y: 3}, {X: 1, Y: 4}, {X: 7, Y: 4}}
	start := kinematics.Pose{X: 1, Y: 1}
	t.Logf("[local-free] 输入 start=%v 路径=%v", start, path)
	end, ok := rollout(t, lp, start, path, g, nil)
	t.Logf("[local-free] 结果 end=%v 到达=%v", end, ok)
	if !ok {
		t.Fatal("expected to arrive without obstacles")
	}
}

func TestLocalDynamicObstacleBlocksGap(t *testing.T) {
	g := twoGapMap()
	lp := New(testCfg(), kinematics.DefaultConstraints())
	path := []grid.Cell{{X: 1, Y: 1}, {X: 1, Y: 2}, {X: 1, Y: 3}, {X: 1, Y: 4}, {X: 7, Y: 4}}
	start := kinematics.Pose{X: 1, Y: 1}
	obs := []Obstacle{{ID: "m", X: 7, Y: 4, Radius: 0.7, Seq: 1}}
	t.Logf("[local-blocked] 输入 start=%v 障碍=%v", start, obs)
	_, ok := rollout(t, lp, start, path, g, obs)
	t.Logf("[local-blocked] 判定依据：缺口被圆形障碍封死，局部窗口无合法指令 => 到达=%v（期望false）", ok)
	if ok {
		t.Fatal("expected blockage when gap is occupied by moving obstacle")
	}
}

func TestLocalDeterministic(t *testing.T) {
	g := twoGapMap()
	path := []grid.Cell{{X: 1, Y: 1}, {X: 2, Y: 1}, {X: 7, Y: 0}, {X: 13, Y: 0}}
	start := kinematics.Pose{X: 1, Y: 1}
	obs := []Obstacle{{ID: "m", X: 3, Y: 3, Radius: 0.5, Seq: 1}}
	var ends [2]kinematics.Pose
	for i := 0; i < 2; i++ {
		lp := New(testCfg(), kinematics.DefaultConstraints())
		cur := start
		goal := path[len(path)-1]
		for it := 0; it < lp.MaxIters() && !lp.Arrived(cur, goal); it++ {
			traj, blocked, _ := lp.Step(cur, path, g, obs)
			if blocked {
				break
			}
			cur = traj.End()
		}
		ends[i] = cur
	}
	t.Logf("[local-determ] 两次运行末端: %+v / %+v 相等=%v", ends[0], ends[1], ends[0] == ends[1])
	if ends[0] != ends[1] {
		t.Fatal("DWA runs must be bitwise identical")
	}
}

func TestNewRejectsBadConfig(t *testing.T) {
	if New(Config{}, kinematics.DefaultConstraints()) != nil {
		t.Fatal("bad config must yield nil planner")
	}
	if New(testCfg(), kinematics.Constraints{}) != nil {
		t.Fatal("zero constraints must yield nil planner")
	}
	t.Log("[local-config] 非法 Config/约束返回 nil")
}

func TestStepMissingMap(t *testing.T) {
	lp := New(testCfg(), kinematics.DefaultConstraints())
	_, _, err := lp.Step(kinematics.Pose{}, []grid.Cell{{X: 0, Y: 0}, {X: 1, Y: 0}}, nil, nil)
	t.Logf("[local-nomap] err=%v 是ErrNoMap=%v", err, err == ErrNoMap)
	if err != ErrNoMap {
		t.Fatalf("want ErrNoMap got %v", err)
	}
}
