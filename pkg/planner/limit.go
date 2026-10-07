package planner

import "time"

// Clock 抽象“当前时间”读数，用于把墙上时钟超时与真实系统时钟解耦：
// 生产环境用 SystemClock；确定性测试/重放用 ManualClock 手动推进虚拟时间。
type Clock interface {
	NowMicros() int64
}

// SystemClock 使用真实墙上时钟（time.Now）。
type SystemClock struct{}

// NowMicros 返回当前的 Unix 微秒时间。
func (SystemClock) NowMicros() int64 { return time.Now().UnixMicro() }

// ManualClock 是确定性的虚拟时钟：NowMicros 每次被调用都按固定步长推进，
// 因此“超时”结论只取决于调用次数（搜索规模），与机器负载无关。
type ManualClock struct {
	t   int64
	inc int64
}

// NewManualClock 创建步长为 incMicros（<=0 时取 1）的虚拟时钟。
func NewManualClock(incMicros int64) *ManualClock {
	if incMicros <= 0 {
		incMicros = 1
	}
	return &ManualClock{t: 0, inc: incMicros}
}

// NowMicros 返回虚拟当前时间并推进一个步长。
func (c *ManualClock) NowMicros() int64 {
	v := c.t
	c.t += c.inc
	return v
}

// Limit 描述一次规划的时限配置。
//
// 双层兜底：
//   - MaxExpansions 是确定性的节点展开预算（>0 生效），不依赖时钟，
//     是确定性重放与跨机器一致结论的主要“时限”；
//   - DeadlineMicros 是墙上时钟预算（>0 生效），仅作为超大/极端地图下
//     阻止单次搜索无限运行的实时兜底；它不参与确定性复现语义，
//     默认（<=0）关闭，同一份输入不会因机器负载不同而结论漂移。
type Limit struct {
	MaxExpansions  int
	DeadlineMicros int64
	Clock          Clock
}

// NewWithLimit 创建带确定性展开预算与可选墙上时钟时限的规划器。
// DeadlineMicros>0 但 Clock 为 nil 时使用真实墙上时钟（SystemClock）。
func NewWithLimit(lim Limit) *Planner {
	p := New(lim.MaxExpansions)
	if lim.DeadlineMicros > 0 {
		p.deadline = lim.DeadlineMicros
		if lim.Clock != nil {
			p.clock = lim.Clock
		} else {
			p.clock = SystemClock{}
		}
	}
	return p
}
