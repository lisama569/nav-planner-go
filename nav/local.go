package nav

import (
	"fmt"
	"math"
)

// LocalAvoider 基于（叠加了最新动态障碍的）地图与当前轨迹做局部冲突检查。
//
// 策略（解释性优先，而非随机绕行）：
//   - 前瞻窗由 cfg.LocalLookahead（米）确定；
//   - 沿轨迹点累积弧长，进入窗内后逐点查询栅格占据；
//   - 前瞻窗内一旦出现冲突即判定 Blocked=true，交由全局层重规划。
//     冲突之后出现的自由点属于旧轨迹的几何，不能保证机器人能够
//     “连续且无碰撞地”到达，因此不得据此静默放行或改写轨迹；
//   - FreeAfter 仅作为可解释的诊断信息保留。
type LocalAvoider struct {
	cfg Config
}

// NewLocalAvoider 创建局部避障器。
func NewLocalAvoider(cfg Config) *LocalAvoider { return &LocalAvoider{cfg: cfg} }

// LocalVerdict 描述局部检查结论。
type LocalVerdict struct {
	Blocked      bool
	Reason       string
	ConflictCell Cell
	ConflictArc  float64 // 首个冲突点距轨迹起点的弧长（米）；无冲突为 -1
	ConflictAt   int     // 首个冲突轨迹点下标；-1 表示无冲突
	FreeAfter    int     // 冲突点之后窗内自由点数量
}

// Check 判断从机器人当前位姿 from 起、前瞻窗内的轨迹段是否与最新障碍冲突。
// from 用于跳过机器人已经走过的轨迹（旧走廊），只评估前方路径；
// dyn 仅用于解释（报告相关动态障碍数量），真正的占据以 m 为准，
// 从而保证“全局/局部使用同一张世界表示”，结论一致。
func (a *LocalAvoider) Check(m *GridMap, traj *Trajectory, from Pose, dyn map[string]Obstacle) LocalVerdict {
	if m == nil {
		return LocalVerdict{Blocked: true, ConflictAt: -1, ConflictArc: -1,
			Reason: "map missing during local check"}
	}
	if traj == nil || len(traj.Points) == 0 {
		return LocalVerdict{Blocked: true, ConflictAt: -1, ConflictArc: -1,
			Reason: "empty trajectory during local check"}
	}

	lookahead := a.cfg.LocalLookahead
	arc := 0.0
	conflictIdx := -1
	conflictArc := -1.0
	var conflictCell Cell
	freeAfter := 0
	inWindow := false

	for i := 1; i < len(traj.Points); i++ {
		prev, cur := traj.Points[i-1], traj.Points[i]
		segLen := math.Hypot(cur.Pose.X-prev.Pose.X, cur.Pose.Y-prev.Pose.Y)
		arc += segLen

		// 定位当前位姿：以“from 与轨迹点的最小距离”确定已走弧长，
		// 到达最近点之前的历史段一律跳过（机器人不可能再撞上身后障碍）。
		if !inWindow {
			dPrev := math.Hypot(prev.Pose.X-from.X, prev.Pose.Y-from.Y)
			dCur := math.Hypot(cur.Pose.X-from.X, cur.Pose.Y-from.Y)
			if dCur <= dPrev {
				// 当前位姿还在该段前方，继续累积但从 0 弧长计窗。
				arc = 0
				continue
			}
			inWindow = true
			arc = dCur // 从当前位置到本点的前向距离
		}

		if arc > lookahead {
			break // 超出前瞻窗，后面的点本轮不关心
		}
		cell := m.WorldToCell(cur.Pose.X, cur.Pose.Y)
		blockedHere := !m.InBounds(cell) || m.IsBlocked(cell)
		switch {
		case blockedHere && conflictIdx < 0:
			conflictIdx = i
			conflictArc = arc
			conflictCell = cell
		case !blockedHere && conflictIdx >= 0:
			freeAfter++
		}
	}

	if conflictIdx < 0 {
		return LocalVerdict{Blocked: false, ConflictAt: -1, ConflictArc: -1,
			Reason: fmt.Sprintf("no conflict within lookahead window %.2fm", lookahead)}
	}
	v := LocalVerdict{
		ConflictCell: conflictCell,
		ConflictAt:   conflictIdx,
		ConflictArc:  conflictArc,
		FreeAfter:    freeAfter,
	}
	// 旧轨迹走廊被侵入即必须重规划；冲突后的自由点不可作为放行依据。
	v.Blocked = true
	v.Reason = fmt.Sprintf(
		"blocked: conflict at traj point #%d cell %s arc=%.3fm with %d dynamic obstacle(s) tracked within lookahead %.2fm (free points after conflict=%d, diagnostic only) -> global replan required",
		conflictIdx, conflictCell, conflictArc, len(dyn), lookahead, freeAfter)
	return v
}
