package nav

import (
	"testing"
	"time"
)

func TestSearchOpenMap(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	p := NewGlobalPlanner(cfg)
	start := Pose{X: 1.0, Y: 1.0}
	goal := Pose{X: 4.0, Y: 3.0}
	path, err := p.Search(m, start, goal, nil, nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(path.Nodes) < 2 {
		t.Fatalf("path too short: %d", len(path.Nodes))
	}
	first := path.Nodes[0].C
	last := path.Nodes[len(path.Nodes)-1].C
	if first != m.WorldToCell(start.X, start.Y) {
		t.Fatalf("path first cell %s != start cell", first)
	}
	if last != m.WorldToCell(goal.X, goal.Y) {
		t.Fatalf("path last cell %s != goal cell", last)
	}
	t.Logf("open map path length=%d nodes=%v", len(path.Nodes), path.Nodes)
}

func TestSearchDeterministic(t *testing.T) {
	cfg := testConfig()
	p := NewGlobalPlanner(cfg)
	start, goal := Pose{X: 0.5, Y: 0.5}, Pose{X: 7.5, Y: 8.0}
	var prev *Path
	for i := 0; i < 5; i++ {
		m := NewGridMap(cfg.Grid)
		// 随机感的墙（固定数据），检验 map 迭代不影响结果。
		for r := 3; r < 15; r += 2 {
			m.SetBlocked(Cell{r, 5 + (r % 3)}, true)
		}
		path, err := p.Search(m, start, goal, nil, nil)
		if err != nil {
			t.Fatal(err)
		}
		if prev != nil && !sameCells(prev, path) {
			t.Fatalf("non-deterministic path on iteration %d", i)
		}
		prev = path
	}
	t.Logf("deterministic path across 5 runs: %d nodes", len(prev.Nodes))
}

func TestSearchUnreachable(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	// 用一圈围墙把目标围起来。
	gx, gy := 4.0, 4.0
	for _, c := range []Cell{
		{7, 7}, {7, 8}, {7, 9},
		{8, 7}, {8, 9},
		{9, 7}, {9, 8}, {9, 9},
	} {
		m.SetBlocked(c, true)
	}
	_ = gx
	_ = gy
	p := NewGlobalPlanner(cfg)
	_, err := p.Search(m, Pose{X: 1, Y: 1}, Pose{X: 4.0, Y: 4.0}, nil, nil)
	// 围墙中心 (r=8,c=8) 即世界 (4,4)，但中心也被墙围住。
	if pe, ok := AsPlanningError(err); !ok || pe.Code != FailUnreachable {
		t.Fatalf("want FailUnreachable, got %v", err)
	}
	t.Logf("unreachable rejected: %v", err)
}

func TestSearchInvalidStartGoal(t *testing.T) {
	cfg := testConfig()
	m := NewGridMap(cfg.Grid)
	m.SetBlocked(Cell{2, 2}, true)
	p := NewGlobalPlanner(cfg)

	if _, err := p.Search(nil, Pose{}, Pose{}, nil, nil); err == nil {
		t.Fatal("nil map must fail")
	} else if pe, _ := AsPlanningError(err); pe.Code != FailMapMissing {
		t.Fatalf("want FailMapMissing, got %v", err)
	}
	if _, err := p.Search(m, Pose{X: 0.5, Y: 0.5}, Pose{X: 0.5, Y: 0.5}, nil, nil); err != nil {
		t.Fatalf("same free cell start=goal must succeed, got %v", err)
	}
	if _, err := p.Search(m, Pose{X: 0.5, Y: 0.5}, Pose{X: 100, Y: 100}, nil, nil); err == nil {
		t.Fatal("out-of-map goal must fail")
	} else if pe, _ := AsPlanningError(err); pe.Code != FailGoalInvalid {
		t.Fatalf("want FailGoalInvalid, got %v", err)
	}
	if _, err := p.Search(m, Pose{X: 0.5, Y: 0.5}, Pose{X: 1.5, Y: 1.5}, nil, nil); err != nil {
		t.Fatalf("free start/goal should succeed: %v", err)
	}
}

func TestSearchTimeout(t *testing.T) {
	cfg := testConfig()
	cfg.PlanTimeout = time.Hour // 让 deadline 本身合法
	m := NewGridMap(cfg.Grid)
	p := NewGlobalPlanner(cfg)
	clk := NewFakeClock(time.Unix(0, 0))
	past := clk.Now()
	_, err := p.Search(m, Pose{X: 1, Y: 1}, Pose{X: 8, Y: 8}, &past, clk)
	if pe, ok := AsPlanningError(err); !ok || pe.Code != FailTimeout {
		t.Fatalf("want FailTimeout with pre-expired deadline, got %v", err)
	}
	t.Logf("timeout rejected deterministically: %v", err)
}

func sameCells(a, b *Path) bool {
	if len(a.Nodes) != len(b.Nodes) {
		return false
	}
	for i := range a.Nodes {
		if a.Nodes[i].C != b.Nodes[i].C {
			return false
		}
	}
	return true
}
