#include <collision_matrix.h>
#include <PlanningScene.h>


namespace franka_hw {
  moveit_msgs::PlanningScene planning_scene;
    collision_detection::AllowedCollisionMatrix& acm = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrixNonConst();
    acm.setEntry(link1, link2, true);
    acm.getMessage(planning_scene.allowed_collision_matrix);
    planning_scene.is_diff = true;
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client_.call(srv);
}
