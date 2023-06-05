# Behavior Tree

> 研一下机器人仿真课设，由于 SMACC 团队对于俄罗斯 IP 偏爱有加，我没有好的状态机库可用。于是尝试用行为树代替状态机。
> 
> 软件版本：
> 
> BehaviorTree.CPP v3.8
> 
> Groot 1
> 
> 参考：
> 
> https://www.behaviortree.dev/
> 
> Dknt 2023.5.20

BehaviorTree 提供了基于 Qt 的可视化工具 Groot，编译过程中会报一大堆错，真好。按照 Github issue 更改源码，可以成功编译。[Adapt to API changes in BehaviorTree by galou · Pull Request #172 · BehaviorTree/Groot · GitHub](https://github.com/BehaviorTree/Groot/pull/172)

> 注意：
> 
> BehaviorTree 和 Groot 都可以用 catkin 进行编译，注意版本！
> 
> Groot 编译时要修改对应 CMakeLists 中的 C++ 版本为14以上。
> 
> Qt, Qt, Qt...

# 1 行为树基础

行为树和状态机相似，是一种用于在正确时刻、条件下调用回调函数的工具，用于机器人、游戏 AI 等逻辑控制场景。

## 1.1 节点类型

* `ControlNode` 控制节点 可以有多个子节点，按照一定策略执行动作

* * `Sequence` 顺序执行
  
  * `Fallbacks` 应变执行。当子节点返回失败时，执行后续子节点；只要一个子节点返回成功，则返回成功。

* `DecoratorNode`  修饰节点 只有一个子节点，实现某种逻辑

* * `InverterNode` 结果取反
  
  * `ForceSuccess` 强制成功
  
  * `ForceFailure` 强制失败
  
  * `Repeat` 重复
  
  * `Retry` 多次尝试

* `ConditionNode`

* `ActionNode`

* * `SyncActionNode` 同步动作

行为树由若干上述节点组成，编写代码时，需要从标准库中继承类，并重载行为函数。

`Blackboard`作为全局参数存储区

`Ports`节点之间通信的工具

# 2 BT库使用

## 2.1 XML树文件

使用 XML 文件保存树结构，运行时动态加载树。XML 树结构加载通过`BehaviorTreeFactory`对象实现。

有紧凑和显式两种表示方法，如果使用 Groot，必须显示编写 XML 文件，或在 XML 给出一些额外信息。

> 给出额外信息的方法好像不太好用，还是显示表示吧

示例：

```xml
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Condition ID="CheckBattery"   name="check_battery"/>
            <Action ID="OpenGripper"    name="open_gripper"/>
            <Action ID="ApproachObject" name="approach_object"/>
            <Action ID="CloseGripper"   name="close_gripper"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## 2.2 节点

`BehaviorTreeFactory`使用示例：

```cpp
#include <behaviortree_cpp_v3/bt_factory.h>
using namespace BT;
BehaviorTreeFactory factory;

// 以下节点中调用的回调函数，可以有三种形式
// 动作节点  需要写 ApproachObject 节点类
factory.registerNodeType<ApproachObject>("ApproachObject");
// 简单条件节点  需要写 CheckBattery 回调函数
factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode& self){ return CheckBattery(); });
// 简单动作节点  需要写 open 回调函数
factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode& self){ return open(); } );
// 从文件读取树
auto tree = factory.createTreeFromFile("./my_tree.xml");
// 执行
tree.tickRootWhileRunning();
```

> 注意 lambda 表达式参数列表中必须有`BT::TreeNode&`，否则编译报错。官网例程写的有问题。

# Groot

> I'm Groot???

如果使用 catkin 编译，终端运行：

```shell
rosrun groot Groot
```
