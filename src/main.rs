use rapier2d::prelude::*;

fn simulate(mut rigid_body_set: RigidBodySet, mut collider_set: ColliderSet, mut joint_set: ImpulseJointSet, thigh_handle: RigidBodyHandle, calf_handle: RigidBodyHandle, num_frames: u32, gravity: Option<Vector<f32>>) {
    let gravity = gravity.unwrap_or_else(|| { vector![0.0, -9.81] });
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..num_frames {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        let thigh_body = &rigid_body_set[thigh_handle];
        let calf_body = &rigid_body_set[calf_handle];

        println!("Thigh altitude: {}", thigh_body.translation().y);
        println!("Calf altitude: {}", calf_body.translation().y);
    }
}

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut joint_set = ImpulseJointSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1).build();
    collider_set.insert(collider);

    /* Create the leg parts. */
    let thigh_length = 10.;
    let calf_length = 10.;
    let start_height_offset = thigh_length / 2.;
    let knee_damp = 1.;

    let thigh = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, thigh_length/2. + calf_length + start_height_offset])
        .rotation(0.)
        .build();
    let collider = ColliderBuilder::cuboid(0.1, 5.0).restitution(0.7).build();
    let thigh_handle = rigid_body_set.insert(thigh);
    collider_set.insert_with_parent(collider, thigh_handle, &mut rigid_body_set);

    let calf = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, calf_length/2. + start_height_offset])
        .rotation(0.)
        .build();
    let collider = ColliderBuilder::cuboid(0.1, 5.0).restitution(0.7).build();
    let calf_handle = rigid_body_set.insert(calf);
    collider_set.insert_with_parent(collider, calf_handle, &mut rigid_body_set);

    let knee = RevoluteJointBuilder::new()
        .local_anchor1(point![0.0, -thigh_length/2.])
        .local_anchor2(point![0.0, calf_length/2.])
        .motor_velocity(0., knee_damp);
    joint_set.insert(thigh_handle, calf_handle, knee, true);

    /* Create other structures necessary for the simulation. */
    simulate(rigid_body_set, collider_set, joint_set, thigh_handle, calf_handle, 400, Some(vector![0.0, -9.81]));
}