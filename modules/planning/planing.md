
Planning 模块负责整个车辆的驾驶决策（整合了决策和规划两个功能），而驾驶决策需要根据当前所处的地理位置，周边道路和交通情况来决定，所以 Planning 模块的上游模块是 Localization、Prediction、Routing 模块。Planning 不直接控制车辆硬件，因此 Planning 的下游模块是 Control 模块。Routing 模块会先规划出一条导航线路，然后 Planning 模块根据这条路线做局部优化，如果 Planning 模块发现 Routing 规划的路线行不通（Routing 只是根据地图规划路线，并不考虑实际情况），会触发 Routing（给 Routing 发送一个消息）重新规划路线，因此，Planning 和 Routing 两个模块的数据流是双向的。


Planning 模块主要的责任是：根据导航信息以及车辆的当前状态，在有限的时间范围内计算出一条合适的轨迹供车辆行驶。

1. 车辆的行驶路线通常由 Routing 模块提供，Routing 模块会根据目的地以及地图搜索出一条代价尽可能小的路线。
2. 车辆的当前状态包含了很多因素，例如：车辆自身的状态（包括姿态，速度，角速度等），当前所处位置，周边物理世界的静态环境以及交通状态等等。
3. Planning 模块的响应速度必须是稳定可靠的，正常人类的反应速度是 300 ms，车辆的反应速度要小于 100 ms，基于此 Planning 模块需要以 10 Hz 的频率运行着，如果其中一个模块时间过长，会影响其他模块
4. “合适的轨迹”有多个层次的含义，“轨迹”不同于路径，“轨迹”不仅包含了行驶路线，嗨哟啊包含每个时刻的车辆的速度，加速度，方向盘转向等信息。其次，这条轨迹必须是底层控制可以执行的，因为车辆再运动过程中具有一定的惯性，车辆的转弯角速度也是有限制的，同时还要考虑乘坐人员的体验。



Apollo FSM（Finite State Machine，Finite 有限，Infinite 无限）：一个有限状态机，与高精地图确定车辆状态，给定其位置和路线。

Planning Dispatcher（dispatcher n调度器）：根据车辆的状态和其他相关信息调度合适的 Planner（planner n 计划着，规划期）。


Decider 和 Optimizers：一组实现决策任务和各种优化的无状态库。
- Decider：建议何时环道、何时停车、何时爬行（慢速行进）或爬行合适完成
- Optimizers：车辆的轨迹和速度
Apollo 中的 Task 分为 Decider 、Optimizers、learning_model 三个子类。


## 整体Pipeline


- Planning and Control Map:
	- PncMap::UpdateRoutingResponse
	- PncMap::GetRouteSegments
- Frame:
	- ReferenceLineProvider::GetReferenceLines
	- TrafficDecider::Execute(StdPlanning)
	- ReferenceLineInfo::AddObstacle(TrafficRule)
	- ScenarioManager::Update(PublicRoadPlanner)
- EM Planner
	- Reference Line Generator
		- Line 1
			- Reference Line Frenet Frame
			- Optimizer
				- SL Projection(E-step)
				- Path Planning(M-step)
				- ST Projection(E-step)
				- Speed Planning(M-step)
		- Line 2
			- Optimizer
				- SL Projection(E-step)
				- Path Planning(M-step)
				- ST Projection(E-step)
				- Speed Planning(M-step)
	- Reference Line Trajectory Decider
	- Fessible car Trajectory

```c++
1. **SL Projection (E-step):**
    
    - SL可能指的是Space (空间) 和 Lane (车道)。因此，SL Projection可能是在空间和车道方向上进行的投影。
    - E-step可能表示期望（Expectation）步骤，这可能涉及到对某些状态或变量的期望计算或估计。
2. **Path Planning (M-step):**
    
    - Path Planning通常指的是生成车辆沿着道路的路径的过程，以达到特定目标。
    - M-step可能表示最大化（Maximization）步骤，可能涉及优化路径规划的某些目标或准则。
3. **ST Projection (E-step):**
    
    - ST可能指的是Space (空间) 和 Time (时间)。因此，ST Projection可能是在空间和时间方向上进行的投影。
    - E-step可能同样表示期望（Expectation）步骤，可能涉及到对某些状态或变量的期望计算或估计。
4. **Speed Planning (M-step):**
    
    - Speed Planning通常指的是为车辆规划适当的速度轨迹，以便安全、高效地沿着路径行驶。
    - M-step同样可能表示最大化（Maximization）步骤，可能涉及优化速度规划的某些目标或准则。

这些术语可能属于某种路径规划或自动驾驶算法的框架或流程的一部分，其中E-step和M-step可能是期望最大化（Expectation-Maximization，EM）算法中常见的步骤。具体的含义可能取决于上下文和具体的路径规划算法。
```

PncMap（Planning and Control Map）：根据 Routing 提供的数据，生成 Planning 模块需要的路径信息。

Frame：Frame 是向 Planning 提供数据的。Frame 中包含了 Planning 一次计算循环中需要的所有数据。例如：地图，车辆状态，参考线，障碍物信息等。ReferenceLien 是车辆行驶的参考线，TrafficDecider 与交通规则相关。

EM（Expectation-Maximization，最大期望） Planner：Apollo系统中内置了好几个Planner，但目前默认使用的是EM Planner，这也是专门为开放道路设计的。


## Planner

## Planning 和 Planner

Apollo 3.5 废弃了原先的 ROS，引入了新的运行环境：Cyber RT。
Cyber RT 以组件的方式来管理各个模块，组件的实现会基于该框架提供的基类：apollo::cyber::Component。

Planning 模块自然也不例外，其实现类是下面这个：

```c++
class PlanningComponent final : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis, localization::LocalizationEstimate>
```

在 PlanningComponent 的实现中，会根据具体的配置选择 Planning 的入口。Planning 的入口通过 PlanningBase 类来描述的。

PlanningBase（规划基类） 只是一个抽象类，该类有三个子类：
- OpenSpacePlanning（开放空间规划器，例如：泊车）
- NaviPlanning（导航规划器，例如：高速路行驶）
- StdPlanning（标准规划器）

PlanningComponent::Init()方法会根据配置选择具体的 Planning 入口：

```c++
bool PlanningComponent::Init()
{
  if (FLAGS_open_space_planner_switchable)
  {
    planning_base_ = std::make_unique<OpenSpacePlanning>();
  } 
  else 
  {
    if (FLAGS_use_navigation_mode) 
    {
      planning_base_ = std::make_unique<NaviPlanning>();
    } 
    else 
	{
      planning_base_ = std::make_unique<StdPlanning>();
    }
  }
}

// PlanningComponent 有个 planning_base_ 成员
std::unique_ptr<PlanningBase> planning_base_;

// 规划模块的入口函数是 PlanningComponent 的 Proc
```

Planning 模块的主题逻辑是在具体的规划器（PlanningBase 的子类）中的 Run Once 函数中实现的，RunOnce 函数会被 timer 以固定的间隔调用，每次调用就是一个规划周期。

```c++
void StdPlaning::RunOnce(const LocalView& local_view, ADCTrajectory* const trajectory_pb) override;
```


## PublicRoadPlanner

PublicRoadPlanner 是目前默认的 Planner，它实现了 EM（Expectation Maximization）算法。

Planner的算法实现依赖于两个输入：

- 车辆自身状态：通过**TrajectoryPoint**描述。该结构中包含了车辆的位置，速度，加速度，方向等信息。
- 当前环境信息：通过`Frame`描述。前面我们已经提到，`Frame`中包含了一次Planning计算循环中的所有信息。

在`Frame`中有一个数据结构值得我们重点关于一下，那就是`LocalView`。这个类在前面我们也已经提到过。它的定义如下：

```c++
struct LocalView 
{
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<routing::RoutingResponse> routing;
  bool is_new_routing = false;
  std::shared_ptr<relative_map::MapMsg> relative_map;
};
```

从这个定义中可以看到，这个结构中包含了这些信息：

- 障碍物的预测信息
- 车辆底盘信息
- 大致定位信息
- 交通灯信息
- 导航路由信息
- 相对地图信息

对于每个Planner（规划器）来说，其主要的逻辑都实现在`Plan`方法中。`PublicRoadPlanner::Plan`方法的实现逻辑如下：

``` c++
Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame) 
{
  DCHECK_NOTNULL(frame);
  scenario_manager_.Update(planning_start_point, *frame); ①
  scenario_ = scenario_manager_.mutable_scenario(); ②
  auto result = scenario_->Process(planning_start_point, frame); ③

  ...
  if (result == scenario::Scenario::STATUS_DONE) 
  {
    scenario_manager_.Update(planning_start_point, *frame); ④
  } 
  else if (result == scenario::Scenario::STATUS_UNKNOWN) 
  {
    return Status(common::PLANNING_ERROR, "scenario returned unknown");
  }
  return Status::OK();
}
```

这段代码的几个关键步骤是：

1. 确定当前Scenario：因为Frame中包含了当前状态的所有信息，所以通过它就可以确定目前是处于哪一个场景下。
2. 获取当前Scenario。
3. 通过Scenario进行具体的处理。
4. 如果处理成功，则再次通过ScenarioManager更新。


# Scenario 

## 场景分类


### 车道保持

车道保持场景是默认的驾驶场景，它不仅仅包含单车道巡航。同时也包含了：

- 换道行驶
- 遵循基本的交通约定
- 基本转弯

### Side Pass（侧通，绕行）

在这种情况下，如果在自动驾驶车辆（ADC）的车道上有静态车辆或静态障碍物，并且车辆不能在不接触障碍物的情况下安全地通过车道，则执行以下策略：

- 检查邻近车道是否接近通行
- 如果无车辆，进行绕行，绕过当前车道进入邻道
- 一旦障碍物安全通过，回到原车道上


### 停止标识

停止标识有两种分离的驾驶场景：

1、未保护：在这种情况下，汽车预计会通过具有双向停车位的十字路口。因此，我们的ADC必须爬过并测量十字路口的交通密度，然后才能继续走上它的道路。
2、受保护：在此场景中，汽车预期通过具有四向停车位的十字路口导航。我们的ADC将必须对在它之前停下来的汽车进行测量，并在移动之前了解它在队列中的位置。


## 场景实现

场景的实现主要包含三种类：

- `ScenarioManager`：场景管理器类。负责注册，选择和创建`Scenario`。
- `Scenario`：描述一个特定的场景（例如：Side Pass）。该类中包含了`CreateStage`方法用来创建`Stage`。一个Scenario可能有多个Stage对象。在Scenario中会根据配置顺序依次调用`Stage::Process`方法。该方法的返回值决定了从一个Stage切换到另外一个Stage。
- `Stage`：如上面所说，一个Scenario可能有多个Stage对象。场景功能实现的主体逻辑通常是在`Stage::Process`方法中。

## 场景配置

```c++
// proto/planning_config.proto
message ScenarioConfig {
  
  message StageConfig {
    optional StageType stage_type = 1;
    optional bool enabled = 2 [default = true];
    repeated TaskConfig.TaskType task_type = 3;
    repeated TaskConfig task_config = 4;
  }

  optional ScenarioType scenario_type = 1;
  oneof scenario_config {
    ScenarioLaneFollowConfig lane_follow_config = 2;
    ScenarioSidePassConfig side_pass_config = 3;
    ScenarioStopSignUnprotectedConfig stop_sign_unprotected_config = 4;
    ScenarioTrafficLightProtectedConfig traffic_light_protected_config = 5;
    ScenarioTrafficLightUnprotectedRightTurnConfig traffic_light_unprotected_right_turn_config = 6;
  }

  repeated StageType stage_type = 7;
  repeated StageConfig stage_config = 8;
}
```


这里定义了ScenarioConfig结构，一个ScenarioConfig中可以包含多个StageConfig。

另外，Stage和Scenario都有一个Type字段，它们的定义如下：

```c++
enum ScenarioType 
{
    LANE_FOLLOW = 0;  // default scenario
    CHANGE_LANE = 1;
    SIDE_PASS = 2;  // go around an object when it blocks the road
    APPROACH = 3;   // approach to an intersection
    STOP_SIGN_PROTECTED = 4;
    STOP_SIGN_UNPROTECTED = 5;
    TRAFFIC_LIGHT_PROTECTED = 6;
    TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN = 7;
    TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN = 8;
}
  
enum StageType 
{
    NO_STAGE = 0;
    
    LANE_FOLLOW_DEFAULT_STAGE = 1;
    
    STOP_SIGN_UNPROTECTED_PRE_STOP = 100;
    STOP_SIGN_UNPROTECTED_STOP = 101;
    STOP_SIGN_UNPROTECTED_CREEP = 102 ;
    STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE = 103;
    
    SIDE_PASS_APPROACH_OBSTACLE = 200;
    SIDE_PASS_GENERATE_PATH= 201;
    SIDE_PASS_STOP_ON_WAITPOINT = 202;
    SIDE_PASS_DETECT_SAFETY = 203;
    SIDE_PASS_PASS_OBSTACLE = 204;
    SIDE_PASS_BACKUP = 205;
    
    TRAFFIC_LIGHT_PROTECTED_STOP = 300;
    TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE = 301;
    
    TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_STOP = 310;
    TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP = 311 ;
    TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE = 312;
};
```

## 场景注册

前面我们已经提到，`ScenarioManager`负责场景的注册。实际上，注册的方式就是读取配置文件：

```c++
void ScenarioManager::RegisterScenarios() 
{
  CHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                             &config_map_[ScenarioConfig::LANE_FOLLOW]));
  CHECK(Scenario::LoadConfig(FLAGS_scenario_side_pass_config_file,
                             &config_map_[ScenarioConfig::SIDE_PASS]));
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_stop_sign_unprotected_config_file,
      &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_protected_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
  CHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));
}
```

配置文件在上文中已经全部列出。很显然，这里读取的配置文件位于`/modules/planning/conf/scenario`目录下。


## 场景确定

下面这个函数用来确定当前所处的场景。前面我们已经说了，确定场景的依据是`Frame`数据。

```
void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) 
{
  for (auto scenario : scenario_list_) 
  {
    // 遍历所有的场景，根据当前场景判断是否切换到新的场景
    scenario->IsTransferable(current_scenario_.get(), *frame)
  }
}
```

这里面的逻辑就不过多说明了，读者可以自行阅读相关代码。

## 场景配置

场景的配置文件都位于`/modules/planning/conf/scenario`目录下。在配置场景的时候，还会同时为场景配置相应的Task对象。关于这部分内容，在下文讲解Task的时候再一起看。

# ReferenceLine

从这个图上我们可以看出，参考线是整个决策规划算法的基础。从前面的内容我们也看到了，在Planning模块的每个计算循环中，会先生成 ReferencePath，然后在这个基础上进行后面的处理。例如：把障碍物投影到参考线上。

在下面的内容，我们把详细代码贴出来看一下。



## ReferenceLineProvider

ReferenceLine由 ReferenceLineProvider 专门负责生成。这个类的结构如下：

```c++
很多。。。。
```
### 创建ReferenceLine（不是路径，而是轨迹，包含速度等信息）

ReferenceLine是在`StdPlanning::InitFrame`函数中生成的，相关代码如下：

```c++
Status StdPlanning::InitFrame(const uint32_t sequence_num,
                              const TrajectoryPoint& planning_start_point,
                              const double start_time,
                              const VehicleState& vehicle_state,
                              ADCTrajectory* output_trajectory) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         start_time, vehicle_state,
                         reference_line_provider_.get(), output_trajectory));
  ...

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
```


## ReferenceLineInfo

在ReferenceLine之外，在common目录下还有一个结构：ReferenceLineInfo，这个结构才是各个模块实际用到数据结构，它其中包含了ReferenceLine，但还有其他更详细的数据。

从ReferenceLine到ReferenceLineInfo是在`Frame::CreateReferenceLineInfo`中完成的。


```c++
bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }
  ...
```

ReferenceLineInfo不仅仅包含了参考线信息，还包含了车辆状态，路径信息，速度信息，决策信息以及轨迹信息等。Planning模块的算法很多都是基于ReferenceLineInfo结构完成的。例如下面这个：

```c++
bool Stage::ExecuteTaskOnReferenceLine(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    ...
    
    if (reference_line_info.speed_data().empty()) {
      *reference_line_info.mutable_speed_data() =
          SpeedProfileGenerator::GenerateFallbackSpeedProfile();
      reference_line_info.AddCost(kSpeedOptimizationFallbackCost);
      reference_line_info.set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
    } else {
      reference_line_info.set_trajectory_type(ADCTrajectory::NORMAL);
    }
    DiscretizedTrajectory trajectory;
    if (!reference_line_info.CombinePathAndSpeedProfile(
            planning_start_point.relative_time(),
            planning_start_point.path_point().s(), &trajectory)) {
      AERROR << "Fail to aggregate planning trajectory.";
      return false;
    }
    reference_line_info.SetTrajectory(trajectory);
    reference_line_info.SetDrivable(true);
    return true;
  }
  return true;
}
```

## Smoother

为了保证车辆轨迹的平顺，参考线必须是经过平滑的，目前Apollo中包含了这么几个Smoother用来做参考线的平滑：


# TrafficRule

行驶在城市道路上的自动驾驶车辆必定受到各种交通规则的限制。在正常情况下，车辆不应当违反交通规则。

另外，交通规则通常是多种条例，不同城市和国家地区的交通规则可能是不一样的。

如果处理好这些交通规则就是模块实现需要考虑的了。目前Planning模块的实现中，有如下这些交通规则的实现：

## TrafficRule配置

交通条例的生效并非是一成不变的，因此自然就需要有一个配置文件来进行配置。交通规则的配置文件是：`modules/planning/conf/traffic_rule_config.pb.txt`。

下面是其中的一个代码片段：

```c++
// modules/planning/conf/traffic_rule_config.pb.txt
...

config: {
  rule_id: SIGNAL_LIGHT
  enabled: true
  signal_light {
    stop_distance: 1.0
    max_stop_deceleration: 6.0
    min_pass_s_distance: 4.0
    max_stop_deacceleration_yellow_light: 3.0
    signal_expire_time_sec: 5.0
    max_monitor_forward_distance: 135.0
    righ_turn_creep {
      enabled: false
      min_boundary_t: 6.0
      stop_distance: 0.5
      speed_limit: 1.0
    }
  }
}
```


## TrafficDecider

TrafficDecider是交通规则处理的入口，它负责读取上面这个配置文件，并执行交通规则的检查。在上文中我们已经看到，交通规则的执行是在`StdPlanning::RunOnce`中完成的。具体执行的逻辑如下：


```c++
Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  for (const auto &rule_config : rule_configs_.config()) {
    if (!rule_config.enabled()) { ①
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config); ②
    if (!rule) {
      continue;
    }
    rule->ApplyRule(frame, reference_line_info); ③
  }

  BuildPlanningTarget(reference_line_info); ④
  return Status::OK();
}
```
这段代码说明如下：

1. 遍历配置文件中的每一条交通规则，判断是否enable。
2. 创建具体的交通规则对象。
3. 执行该条交通规则逻辑。
4. 在ReferenceLineInfo上合并处理所有交通规则最后的结果。


# Task

一直到目前最新的Apollo 3.5版本为止，Planning模块最核心的算法就是其EM Planner（实现类是`PublicRoadPlanner`），而EM Planner最核心的就是其决策器和优化器。

但由于篇幅所限，这部分内容本文不再继续深入。预计后面会再通过一篇文章来讲解。这里我们仅仅粗略的了解一下其实现结构。

Planning中这部分逻辑实现位于`tasks`目录下，无论是决策器还是优化器都是从`apollo::planning::Task`继承的。该类具有下面这些子类：


`Task`类提供了`Execute`方法供子类实现，实现依赖的数据结构就是`Frame`和`ReferenceLineInfo`。

```c++
Status Task::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return Status::OK();
}
```

有兴趣的读者可以通过阅读子类的`Execute`方法来了解算法实现。


## Task配置

上文中我们已经提到，场景和Task配置是在一起的。这些配置在下面这些文件中：

```c++
// /modules/planning/conf/scenario
.
├── lane_follow_config.pb.txt
├── side_pass_config.pb.txt
├── stop_sign_unprotected_config.pb.txt
├── traffic_light_protected_config.pb.txt
└── traffic_light_unprotected_right_turn_config.pb.txt
```

一个Scenario可能有多个Stage，每个Stage可以指定相应的Task，下面是一个配置示例：

