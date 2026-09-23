package nav

import (
	"math"
	"testing"
	"time"
)

// testConfig 返回一套数值友好的标准配置：
// 20x20 栅格，分辨率 0.5m（世界范围 x,y ∈ [-0.25, 9.75]）。
func testConfig() Config {
	return Config{
		Grid: GridConfig{
			Rows:       20,
			Cols:       20,
			Resolution: 0.5,
			OriginX:    0.0,
			OriginY:    0.0,
		},
		Limits: Limits{
			MaxLinearVel:  1.0,
			MinLinearVel:  0.0,
			MaxAngularVel: math.Pi,
			MaxCurvature:  2.0,
			MaxAccel:      2.0,
			MaxSteerRate:  math.Pi,
			MaxStepTheta:  math.Pi / 2,
		},
		DT:             100 * time.Millisecond,
		PlanTimeout:    2 * time.Second,
		GoalTolerance:  0.5,
		Inflation:      0.0,
		LocalLookahead: 2.0,
	}
}

// logResult 按需求打印输入状态、规划结果与判定依据。
func logResult(t *testing.T, tag string, start, goal Pose, r *PlanResult) {
	t.Helper()
	t.Logf("[%s] input: start=%+v goal=%+v", tag, start, goal)
	if r == nil {
		t.Logf("[%s] result: <nil>", tag)
		return
	}
	t.Logf("[%s] result: seq=%d replanned=%v elapsed=%v succeeded=%v",
		tag, r.Seq, r.Replanned, r.Elapsed, r.Succeeded())
	if r.Path != nil {
		t.Logf("[%s] path: %d node(s), first=%v last=%v",
			tag, len(r.Path.Nodes), safeNode(r.Path, 0), safeNode(r.Path, len(r.Path.Nodes)-1))
	}
	if r.Traj != nil {
		t.Logf("[%s] traj: status=%d points=%d reason=%q",
			tag, r.Traj.Status, len(r.Traj.Points), r.Traj.Reason)
	}
	if r.ReplanReason != "" {
		t.Logf("[%s] replan reason: %s", tag, r.ReplanReason)
	}
	if r.Err != nil {
		t.Logf("[%s] rejected: code=%s msg=%s", tag, r.Err.Code, r.Err.Msg)
	}
}

func safeNode(p *Path, i int) any {
	if p == nil || i < 0 || i >= len(p.Nodes) {
		return "-"
	}
	return p.Nodes[i]
}

// assertCode 断言失败分类码可区分且符合预期。
func assertCode(t *testing.T, r *PlanResult, want FailCode) {
	t.Helper()
	if want == FailNone {
		if r.Err != nil {
			t.Fatalf("expected success, got %s", r.Err.Error())
		}
		return
	}
	if r.Err == nil {
		t.Fatalf("expected failure code %s, got success", want)
	}
	if r.Err.Code != want {
		t.Fatalf("expected failure code %s, got %s (%s)", want, r.Err.Code, r.Err.Msg)
	}
}
