#include "soc/context.hpp"

// Phase 1 阶段 LaneWidthEstimator/LeadTracker/... 等管理器类型尚未移植（Phase 2），
// 因此 ControlManagers 的构造/析构/移动特殊成员函数在 soc/context.hpp 中只声明、
// 不在此定义——unique_ptr<IncompleteType> 的默认构造不要求完整类型，但析构/移动赋值
// 需要，留到 Phase 2（届时上述类型完整）在本文件补上定义。
// 只要没有代码构造/销毁 ControlManagers 实例，当前声明-不定义的状态即可正常编译链接。
