package nav

import (
	"testing"
	"time"
)

func TestConfigValidation(t *testing.T) {
	base := testConfig()
	cases := []struct {
		name string
		mut  func(*Config)
		want FailCode
	}{
		{"bad grid", func(c *Config) { c.Grid.Rows = 0 }, FailMapMissing},
		{"zero max vel", func(c *Config) { c.Limits.MaxLinearVel = 0 }, FailConstraint},
		{"positive min vel", func(c *Config) { c.Limits.MinLinearVel = 1 }, FailConstraint},
		{"neg curvature", func(c *Config) { c.Limits.MaxCurvature = -1 }, FailConstraint},
		{"zero dt", func(c *Config) { c.DT = 0 }, FailConstraint},
		{"zero timeout", func(c *Config) { c.PlanTimeout = 0 }, FailTimeout},
		{"neg tolerance", func(c *Config) { c.GoalTolerance = -1 }, FailGoalInvalid},
		{"neg inflation", func(c *Config) { c.Inflation = -1 }, FailObstacleInvalid},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			c := base
			tc.mut(&c)
			err := c.Validate()
			if pe, ok := AsPlanningError(err); !ok || pe.Code != tc.want {
				t.Fatalf("want code %s, got %v", tc.want, err)
			}
		})
	}
	if _, err := NewNavigator(base, nil); err != nil {
		t.Fatalf("valid config must construct navigator: %v", err)
	}
}

// TestNavigatorPlanTimeout 端到端验证：搜索过程中预算耗尽时，
// 规划结果必须被分类为 FailTimeout，且不得产出可下发轨迹。
func TestNavigatorPlanTimeout(t *testing.T) {
	cfg := testConfig()
	cfg.PlanTimeout = time.Millisecond
	clk := NewStepClock(mustTime("2026-09-23T00:00:00Z"), time.Nanosecond,
		func(calls int, c *StepClock) {
			// Now 计数：SetMap=1，Plan 取 t0=2；
			// 从搜索循环的第一次时限检查（calls>=3）起把时钟
			// 确定性地推到 deadline 之后。
			if calls >= 3 {
				c.JumpTo(mustTime("2026-09-23T01:00:00Z"))
			}
		})
	n, err := NewNavigator(cfg, clk)
	if err != nil {
		t.Fatal(err)
	}
	if err := n.SetMap(NewGridMap(cfg.Grid), nil); err != nil {
		t.Fatal(err)
	}
	r := n.Plan(Pose{X: 0.5, Y: 0.5}, Pose{X: 9.0, Y: 9.0})
	logResult(t, "timeout", Pose{X: 0.5, Y: 0.5}, Pose{X: 9.0, Y: 9.0}, r)
	assertCode(t, r, FailTimeout)
	if r.Traj != nil && r.Traj.Valid() {
		t.Fatal("timed-out plan must not carry a dispatchable trajectory")
	}
}

func TestNavigatorRejectsInvalidObstacle(t *testing.T) {
	n, _ := newTestNavigator(t)
	if ok, why := n.ApplyObstacle(Obstacle{ID: "bad", Seq: 1, HalfX: -1}); ok {
		t.Fatalf("invalid obstacle must be rejected: %s", why)
	} else {
		t.Logf("invalid obstacle rejected: %s", why)
	}
	if ok, _ := n.ApplyObstacle(Obstacle{Seq: 1, X: 1, Y: 1, HalfX: 1, HalfY: 1}); ok {
		t.Fatal("empty-id obstacle must be rejected")
	}
	if n.State().Seq != 0 {
		t.Fatalf("rejected updates must not advance seq, got %d", n.State().Seq)
	}
}
