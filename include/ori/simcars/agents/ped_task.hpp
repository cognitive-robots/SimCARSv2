#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <magic_enum.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedTask
{
    enum class TaskOp
    {
        UNKNOWN = -1,
        NOOP = 0,
        MOVE = 1
    } task_op;

    uint64_t task_node_id;

    PedTask(TaskOp task_op = TaskOp::NOOP, uint64_t task_node_id = 0) : task_op(task_op),
        task_node_id(task_node_id) {}

    friend bool operator ==(PedTask const &task_1, PedTask const &task_2)
    {
        return task_1.task_op == task_2.task_op && task_1.task_node_id == task_2.task_node_id;
    }

    friend std::ostream& operator<<(std::ostream& output_stream, const PedTask& task)
    {
        return output_stream << "Task Operation = " << magic_enum::enum_name(task.task_op) << ", " <<
                                "Task Node Id = " << task.task_node_id;
    }
};

}
}
}
