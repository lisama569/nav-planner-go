package nav

import "time"

// Clock 抽象时间来源，便于确定性重放。
// 真实运行使用 WallClock；测试可使用固定步进的 FakeClock。
type Clock interface {
	Now() time.Time
}

// WallClock 使用系统墙钟。
type WallClock struct{}

// Now 返回当前系统时间。
func (WallClock) Now() time.Time { return time.Now() }

// FakeClock 是可手动推进的确定性时钟。
type FakeClock struct {
	t time.Time
}

// NewFakeClock 从给定时刻构造一个假时钟。
func NewFakeClock(start time.Time) *FakeClock { return &FakeClock{t: start} }

// Now 返回假时钟当前时刻。
func (c *FakeClock) Now() time.Time { return c.t }

// Advance 将假时钟向前推进 d。
func (c *FakeClock) Advance(d time.Duration) { c.t = c.t.Add(d) }

// StepClock 是确定性时钟：每次调用 Now 都自动前进步长，并可选触发回调。
// 用于在不依赖墙钟的情况下，让长循环在中途“超时”。
type StepClock struct {
	t      time.Time
	step   time.Duration
	calls  int
	onCall func(calls int, c *StepClock)
}

// NewStepClock 构造从 start 开始、每次 Now 前进 step 的时钟。
// onCall 非零时，在每次 Now 推进后被调用（可借此在第 N 次查询时跳时间）。
func NewStepClock(start time.Time, step time.Duration, onCall func(int, *StepClock)) *StepClock {
	return &StepClock{t: start, step: step, onCall: onCall}
}

// Now 返回当前时刻，并推进一个步长（模拟时间流逝）。
func (c *StepClock) Now() time.Time {
	c.calls++
	c.t = c.t.Add(c.step)
	if c.onCall != nil {
		c.onCall(c.calls, c)
	}
	return c.t
}

// JumpTo 直接把时钟跳到 t（供回调使用）。
func (c *StepClock) JumpTo(t time.Time) { c.t = t }

// Calls 返回 Now 被调用的次数。
func (c *StepClock) Calls() int { return c.calls }
