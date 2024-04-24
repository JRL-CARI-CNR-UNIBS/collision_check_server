# collision_check_server

The package implements a node that exposes a service to easy check of the collision.
This package is connected and completes the package [`ik_solver`](https://nicolapedrocchi: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@github.com/JRL-CARI-CNR-UNIBS/ik_solver).
This allows the user to avoid all the dependencies and the possibility of integrating different collision checkers

The node is commended inline. The main function for the IK collision checker node. The node exposes a service for checking collisions. The scene is updated by subscribing to the `/get_planning_scene service`. The collision checking is performed by the `MoveCollisionChecker` class. This class is inline commented inside the header and cpp files.

The node is named `ik_collision_check`, and the class `MoveCollisionChecker` implements a service named `~collision_check`.

The type of the `~collision_check` is `ik_solver_msgs::CollisionChecking`.