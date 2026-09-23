package local

import "errors"

// 局部规划的可区分失败原因。
var (
	// ErrBadConfig 表示局部规划参数非法。
	ErrBadConfig = errors.New("local: invalid configuration")
	// ErrNoMap 在需要栅格校验却传入 nil 地图时返回。
	ErrNoMap = errors.New("local: map is missing")
)
