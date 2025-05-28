use rapier2d::prelude::*;
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::camera::Camera;

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

        println!("Thigh position: {}, {}", thigh_body.translation().x, thigh_body.translation().y);
        println!("Calf position: {}, {}", calf_body.translation().x, calf_body.translation().y);
    }
}

fn make_leg(mut rigid_body_set: &mut RigidBodySet, mut collider_set: &mut ColliderSet, mut joint_set: &mut ImpulseJointSet, &start_pos: &Vector<f32>, &thigh_length: &f32, &calf_length: &f32, &knee_stiff: &f32, &knee_damp: &f32) -> (RigidBodyHandle, RigidBodyHandle) {
    let thigh = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, thigh_length/2.] + start_pos)
        .rotation(0.)
        .build();
    let collider = ColliderBuilder::cuboid(0.1, thigh_length).restitution(0.7).build();
    let thigh_handle = rigid_body_set.insert(thigh);
    collider_set.insert_with_parent(collider, thigh_handle, &mut rigid_body_set);

    let calf = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, -calf_length/2.] + start_pos)
        .rotation(45.)
        .build();
    let collider = ColliderBuilder::cuboid(0.1, calf_length).restitution(0.7).build();
    let calf_handle = rigid_body_set.insert(calf);
    collider_set.insert_with_parent(collider, calf_handle, &mut rigid_body_set);

    let knee = RevoluteJointBuilder::new()
        .local_anchor1(point![start_pos[1], -thigh_length/2.])
        .local_anchor2(point![start_pos[1], calf_length/2.])
        .motor_position(45., knee_stiff, knee_damp);
    joint_set.insert(thigh_handle, calf_handle, knee, true);

    (thigh_handle, calf_handle)
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
    let start_height_offset = 4.;
    let start_pos: nalgebra::Matrix<f32, nalgebra::Const<2>, nalgebra::Const<1>, nalgebra::ArrayStorage<f32, 2, 1>> = vector![0., calf_length + start_height_offset];
    let knee_stiff = 10.;
    let knee_damp = 1.;

    let (thigh_handle, calf_handle) = make_leg(&mut rigid_body_set, &mut collider_set, &mut joint_set, &start_pos, &thigh_length, &calf_length, &knee_stiff, &knee_damp);

    /* Create other structures necessary for the simulation. */
    simulate(rigid_body_set, collider_set, joint_set, thigh_handle, calf_handle, 500, Some(vector![0.0, -9.81]));
}