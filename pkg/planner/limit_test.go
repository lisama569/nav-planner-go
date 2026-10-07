package planner

import (
	"errors"
	"testing"

	"github.com/lisama569/nav-planner-go/pkg/grid"
)

// TestPlanTimeoutClassified 验证墙上时钟超时是独立、可区分的结论：
// 返回 ErrPlanTimeout（区别于 ErrPlanLimit），且不返回半成品路径。
func TestPlanTimeoutClassified(t *testing.T) {
	g := grid.New(200, 200)
	start, goal := grid.Cell{X: 1, Y: 1}, grid.Cell{X: 198, Y: 198}

	// 展开预算放得很宽，让“超时”而不是“超预算”先发生。
	// 虚拟时钟每次读取推进 1ms：检查粒度 256 次展开读取一次时钟，
	// deadline=2000us => 第 2 次读取（第 512 次展开）判定超时。
	clk := NewManualClock(1000)
	p := NewWithLimit(Limit{MaxExpansions: 1_000_000, DeadlineMicros: 2000, Clock: clk})
	path, err := p.Plan(g, start, goal)
	t.Logf("[timeout] 输入 start=%v goal=%v deadline=2000us(虚拟) expansions=%d",
		start, goal, p.Expansions())
	t.Logf("[timeout] 结果 err=%v 路径长度=%d | 判定依据 errors.Is(ErrPlanTimeout)=%v",
		err, len(path), errors.Is(err, ErrPlanTimeout))
	if !errors.Is(err, ErrPlanTimeout) {
		t.Fatalf("want ErrPlanTimeout got %v", err)
	}
	if errors.Is(err, ErrPlanLimit) {
		t.Fatal("timeout must not be confused with expansion-budget limit")
	}
	if path != nil {
		t.Fatal("timed-out plan must not emit a half-finished path")
	}
}

// TestPlanTimeoutDeterministic 验证虚拟时钟下“超时”结论与触发位置完全可复现，
// 与机器负载无关：相同输入、相同时钟配置，任意次运行都在同一展开数超时。
func TestPlanTimeoutDeterministic(t *testing.T) {
	g := grid.New(200, 200)
	start, goal := grid.Cell{X: 1, Y: 1}, grid.Cell{X: 198, Y: 198}
	run := func() (int, error) {
		p := NewWithLimit(Limit{MaxExpansions: 1_000_000, DeadlineMicros: 2000,
			Clock: NewManualClock(1000)})
		_, err := p.Plan(g, start, goal)
		return p.Expansions(), err
	}
	e1, err1 := run()
	e2, err2 := run()
	t.Logf("[timeout-determinism] 两次运行 expansions=%d,%d err1=%v err2=%v",
		e1, e2, err1, err2)
	if !errors.Is(err1, ErrPlanTimeout) || !errors.Is(err2, ErrPlanTimeout) {
		t.Fatalf("both runs must time out: %v %v", err1, err2)
	}
	if e1 != e2 {
		t.Fatalf("timeout trigger point must be stable: %d vs %d", e1, e2)
	}
}

// TestDeadlineDisabledByDefault 验证默认构造与 DeadlineMicros<=0 都不启用墙上时钟，
// 即使传入虚拟时钟也不应影响搜索结论（确定性重放的边界）。
func TestDeadlineDisabledByDefault(t *testing.T) {
	g := wallMap()
	start, goal := grid.Cell{X: 1, Y: 1}, grid.Cell{X: 6, Y: 4}
	p := NewWithLimit(Limit{MaxExpansions: 0, DeadlineMicros: 0, Clock: NewManualClock(1000)})
	path, err := p.Plan(g, start, goal)
	t.Logf("[deadline-off] deadline=0 => err=%v 路径长度=%d（应正常成功）", err, len(path))
	if err != nil || path == nil {
		t.Fatalf("deadline disabled must behave like ordinary plan: %v", err)
	}
}
