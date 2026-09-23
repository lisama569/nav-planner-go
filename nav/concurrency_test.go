package nav

import (
	"fmt"
	"sync"
	"testing"
)

// TestConcurrentUpdatesNoRace 在 -race 下验证：
// 并发的障碍写入、规划与状态读取不产生数据竞争；
// 最终状态与并发调度无关地等于“每 ID 最大序号消息”所描述的状态。
func TestConcurrentUpdatesNoRace(t *testing.T) {
	n, _ := newTestNavigator(t)
	const writers = 8
	const perWriter = 25
	const ids = 4

	var wg sync.WaitGroup
	start := Pose{X: 0.5, Y: 0.5}
	goal := Pose{X: 8.5, Y: 8.5}

	// 规划者：与写入并发地反复规划，结果必须永不为 nil。
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 60; i++ {
			if r := n.Plan(start, goal); r == nil {
				t.Error("Plan must never return nil")
				return
			}
		}
	}()

	// 读取者：持续抓取快照。
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < 100; i++ {
			s := n.State()
			if s.LastResult != nil {
				_ = s.LastResult.Succeeded()
			}
		}
	}()

	// 写入者：多个 writer 竞争同一组 ID，序号故意交错、补发旧消息。
	for w := 0; w < writers; w++ {
		wg.Add(1)
		go func(w int) {
			defer wg.Done()
			for k := 0; k < perWriter; k++ {
				id := fmt.Sprintf("obs-%d", w%ids)
				seqHi := int64((k+1)*10 + w)
				seqLo := int64(k*10 + w)
				n.ApplyObstacle(Obstacle{ID: id, Seq: seqHi,
					X: 2 + float64(w)*0.3, Y: 2 + float64(k%5)*0.3,
					HalfX: 0.2, HalfY: 0.2})
				// 旧消息无论何时到达都必须被丢弃，不能覆盖新位置。
				n.ApplyObstacle(Obstacle{ID: id, Seq: seqLo,
					X: 9, Y: 9, HalfX: 0.2, HalfY: 0.2})
			}
		}(w)
	}
	wg.Wait()

	// 与调度无关的最终不变量：
	snap := n.State()
	if snap.DynCount != ids {
		t.Fatalf("expected %d live obstacles, got %d", ids, snap.DynCount)
	}
	if snap.Seq < ids || snap.Seq > int64(writers*perWriter*2) {
		t.Fatalf("world seq out of invariant range: %d", snap.Seq)
	}
	// 每个存活障碍的位置必须等于该 ID “最大序号出现消息”的位置：
	// ID w%4 由 writer w%4 与 w%4+4 竞争，后者的序号恒更大。
	for id, o := range snap.Obstacles {
		var base int
		if _, err := fmt.Sscanf(id, "obs-%d", &base); err != nil {
			t.Fatal(err)
		}
		winnerW := base + 4
		wantX := 2 + float64(winnerW)*0.3
		wantY := 2 + float64((perWriter-1)%5)*0.3
		if o.X != wantX || o.Y != wantY {
			t.Fatalf("obstacle %s settled at (%v,%v), want max-seq position (%v,%v)",
				id, o.X, o.Y, wantX, wantY)
		}
	}

	// 稳定后连跑两次规划结果必须完全一致。
	r1 := n.Plan(start, goal)
	r2 := n.Plan(start, goal)
	logResult(t, "race-final-1", start, goal, r1)
	logResult(t, "race-final-2", start, goal, r2)
	if !equalResults(*r1, *r2) {
		t.Fatal("back-to-back plans on stable state must be identical")
	}
	t.Logf("concurrent updates settled: seq=%d obstacles=%d trajPoints=%d",
		snap.Seq, snap.DynCount, len(r1.Traj.Points))
}

// TestConcurrentRemovalConsistency 验证“移除”在任意交织下都遵循序号规则：
// 比已见序号更旧的出现消息不能复活已移除的障碍。
func TestConcurrentRemovalConsistency(t *testing.T) {
	n, _ := newTestNavigator(t)
	var wg sync.WaitGroup
	wg.Add(3)
	go func() {
		defer wg.Done()
		n.ApplyObstacle(Obstacle{ID: "ghost", Seq: 1, X: 3, Y: 3, HalfX: 0.2, HalfY: 0.2})
	}()
	go func() {
		defer wg.Done()
		n.ApplyObstacle(Obstacle{ID: "ghost", Seq: 3, Removed: true})
	}()
	go func() {
		defer wg.Done()
		n.ApplyObstacle(Obstacle{ID: "ghost", Seq: 2, X: 3, Y: 3, HalfX: 0.2, HalfY: 0.2})
	}()
	wg.Wait()

	s := n.State()
	// 合法交织下接受次数取决于相对顺序（1,2,3 全接受 => 3；
	// 3 先到 => 仅 1；1,3,2 => 接受前两个 => 2），故 Seq ∈ {1,2,3}。
	// 但任何交织下最大序号 3 是“移除”，因此障碍最终绝不存活，
	// 且旧出现消息不能把它复活。
	if s.DynCount != 0 {
		t.Fatalf("ghost must not be live regardless of interleaving, count=%d", s.DynCount)
	}
	if s.Seq < 1 || s.Seq > 3 {
		t.Fatalf("seq must be within [1,3] given three messages, got %d", s.Seq)
	}
	t.Logf("ordered removal consistent under concurrency: seq=%d live=%d", s.Seq, s.DynCount)
}
