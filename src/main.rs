use std::f32::consts::PI;
use std::thread;
use std::time::Duration;

use kiss3d::window::Window;

use kiss3d::{nalgebra as kna, scene};
use rapier2d::parry::query::{Ray};
use rapier2d::prelude::*;
use rapier2d::prelude::nalgebra as rna;

use rurel::mdp::{State, Agent};
/*
#[derive(PartialEq, Eq, Hash, Clone)]
struct MyState { leg1_amplitude: f32, leg1_frequency: f32 , leg1_phase: f32}
#[derive(PartialEq, Eq, Hash, Clone)]
struct MyAction { dx: i32, dy: i32 }*/

#[derive(Clone)]
struct LocalCoordFrame {
    origin: rna::Point2<f32>,
    angle: f32,
}

#[derive(PartialEq)]
struct Rhythm {
    amplitude: f32,
    frequency: f32,
    phase: f32,
}

#[derive(PartialEq)]
struct LegRhythm {
    hip1_rhythm: Rhythm,
    hip2_rhythm: Rhythm,
    knee1_rhythm: Rhythm,
    knee2_rhythm: Rhythm,
}

enum ActuatorParams {
    LegRhythmParams
}

#[derive(PartialEq, Eq, Hash, Clone)]
struct LegRhythmParams {
    hip1_amplitude: i32,
    hip1_frequency: i32,
    hip1_phase: i32,
    hip2_amplitude: i32,
    hip2_frequency: i32,
    hip2_phase: i32,
    knee1_amplitude: i32,
    knee1_frequency: i32,
    knee1_phase: i32,
    knee2_amplitude: i32,
    knee2_frequency: i32,
    knee2_phase: i32,
}

#[derive(PartialEq, Eq, Hash, Clone)]
struct LegRhythmAction {
    hip1_amplitude: i32,
    hip1_frequency: i32,
    hip1_phase: i32,
    hip2_amplitude: i32,
    hip2_frequency: i32,
    hip2_phase: i32,
    knee1_amplitude: i32,
    knee1_frequency: i32,
    knee1_phase: i32,
    knee2_amplitude: i32,
    knee2_frequency: i32,
    knee2_phase: i32,
}

enum Actions {
    LegRhythmParams
}

impl LegRhythmAction {
    fn new_zeros() -> Self {
        Self { hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0 }
    }
}

impl LegRhythmParams {
    fn new_zeros() -> Self {
        Self { hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0 }
    }
}

/*
fn value_discrete(t: f32, amplitude: i32, frequency: i32, phase: i32) -> f32 {

}
 */
fn rhythm_angle(t: f32, rhythm: Rhythm) -> f32 {
    rhythm.frequency * t + rhythm.phase
}

fn rhythm_value(t: f32, rhythm: Rhythm) -> f32 {
    rhythm.amplitude * rhythm_angle(t, rhythm).sin()
}

fn ray_local(ranger: &Ranger, frame: &LocalCoordFrame) -> (Ray, rna::Point2<f32>, f32) {
    let ray_origin_world = local_to_world_position(frame.origin, frame.angle, ranger.ray_local_origin);
    let ray_angle_world = frame.angle + ranger.ray_local_angle;
    (Ray::new(ray_origin_world, rna::vector![ray_angle_world.cos(), ray_angle_world.sin()]), ray_origin_world, ray_angle_world)
}

#[derive(Clone)]
struct Ranger {
    /* `ray_local_origin` and `ray_local_angle` are in local coordinates. */
    frame: LocalCoordFrame,
    ray_local_origin: Point<f32>,
    ray_local_angle: f32,
    range: f32,
}

impl Ranger {
    fn ray_trace(&self, query_pipeline: &mut QueryPipeline, rigid_bodies: &RigidBodySet, colliders: &ColliderSet, frame: &LocalCoordFrame) -> (ColliderHandle, f32, f32) {
        let (ray, _, ray_angle_world) = ray_local(self, frame);
        (query_pipeline.cast_ray(rigid_bodies, colliders, &ray, 10000., true, QueryFilter::default()).unwrap().0,
        query_pipeline.cast_ray(rigid_bodies, colliders, &ray, 10000., true, QueryFilter::default()).unwrap().1,
        ray_angle_world)
    }
    
    fn new(query_pipeline: &mut QueryPipeline, rigid_bodies: &RigidBodySet, colliders: &ColliderSet, ray_local_origin: Point<f32>, ray_local_angle: f32, frame: LocalCoordFrame) -> Self {
        let mut ranger = Self{frame: frame.clone(), ray_local_origin, ray_local_angle, range: 0.};
        let (_, range, _) = Ranger::ray_trace(&ranger, query_pipeline, rigid_bodies, colliders, &frame);
        ranger.range = range;
        ranger
    }
}

#[derive(Clone)]
struct Gps {
    /* `origin` and `angle` are in world coordinates. */
    tracked_body_handle: RigidBodyHandle,
    position: rna::Point2<f32>,
    angle: f32,
}

impl Gps {
    fn new(tracked_body_handle: RigidBodyHandle, rigid_bodies: &RigidBodySet) -> Self {
        let tracked_body = rigid_bodies.get(tracked_body_handle).unwrap();
        Self {tracked_body_handle, position: rna::point![tracked_body.translation().x, tracked_body.translation().y], angle: tracked_body.rotation().angle()}
    }

    fn update(&mut self, tracked_body_handle: &RigidBodyHandle, rigid_bodies: &RigidBodySet) {
        let tracked_body = rigid_bodies.get(*tracked_body_handle).unwrap();
        self.position = rna::point![tracked_body.translation().x, tracked_body.translation().y];
        self.angle = tracked_body.rotation().angle();
    }
}

fn get_joint_angle(joint: &ImpulseJoint, rigid_bodies: &RigidBodySet) -> f32 {
    let rb_rot1 = rigid_bodies.get(joint.body1).unwrap().rotation();
    let rb_rot2 = rigid_bodies.get(joint.body2).unwrap().rotation();
    joint.data.as_revolute().unwrap().angle(rb_rot1, rb_rot2)
}

#[derive(Clone)]
struct Angler {
    /* `angle` is the counterclockwise angle of its associated joint. */
    joint_handle: ImpulseJointHandle,
    angle: f32,
}

impl Angler {
    fn new(joint_handle: ImpulseJointHandle, joints: &ImpulseJointSet, rigid_bodies: &RigidBodySet) -> Self {
        Self {joint_handle: joint_handle, angle: get_joint_angle(joints.get(joint_handle).unwrap(), rigid_bodies)}
    }
    
    fn update(&mut self, joints: &ImpulseJointSet, rigid_bodies: &RigidBodySet) {
        self.angle = get_joint_angle(joints.get(self.joint_handle).unwrap(), rigid_bodies)
    }
}

struct SensorSuite {
    gps: Gps,
    rangers: Vec<Ranger>,
    anglers: Vec<Angler>,
}

fn make_policy(sensors: SensorSuite, parameters: LegRhythmParams, ) -> () {

}

#[derive(Clone)]
struct RigidBodyVisual {
    handle: RigidBodyHandle,
    node: scene::SceneNode,
}

#[derive(Clone)]
struct Body {
    head: Head,
    legs: (Leg, Leg),
    hips: (Hinge, Hinge),
}

#[derive(Clone)]
struct Head {
    /* The robot's local coordinate system has `Head`'s origin and x-axis angle for sensor values. */
    handle: RigidBodyHandle,
    radius: f32,
}

#[derive(Clone)]
struct Leg {
    thigh: Rod,
    calf: Rod,
    knee: Hinge,
}

#[derive(Clone)]
struct Hinge {
    handle: ImpulseJointHandle,
    stiff: f32,
    damp: f32,
}

#[derive(Clone)]
struct Rod {
    handle: RigidBodyHandle,
    length: f32,
}

fn local_to_world_position(origin: Point<f32>, angle: f32, position: Point<f32>) -> Point<f32> {
    let angle_sin = angle.sin();
    let angle_cos = angle.cos();
    let rotated: rna::OPoint<f32, nalgebra::Const<2>> = Matrix::new(angle_cos, -angle_sin, angle_sin, angle_cos) * position;
    rna::point![rotated.x + origin.x, rotated.y + origin.y]
}

fn local_to_world_dir(angle: f32, dir: Vector<f32>) -> Point<f32> {
    let angle_sin = angle.sin();
    let angle_cos = angle.cos();
    let rotated = Matrix::new(angle_cos, -angle_sin, angle_sin, angle_cos) * dir;
    rna::point![rotated.x, rotated.y]
}

fn set_motor_position(joint_set: &mut ImpulseJointSet, joint: &mut Hinge, angle_position: f32) {
    if let Some(generic) = joint_set.get_mut(joint.handle, false) {
        if let Some(knee_rev) = generic.data.as_revolute_mut() {
            knee_rev.set_motor_position(angle_position, joint.stiff, joint.damp);
        }
    }
}

fn draw_sensor_rays(scene_window: &mut Window, query_pipeline: &mut QueryPipeline, rigid_bodies: &RigidBodySet, colliders: &ColliderSet, sensor_rangers: &Vec<Ranger>, coord_sys_origin: rna::Point2<f32>, coord_sys_angle: f32) -> Vec<f32> {
    let mut ranges: Vec<f32> = vec![0.; sensor_rangers.len()];
    for (i, ranger) in sensor_rangers.iter().enumerate() {
        let (_, ray_length, ray_angle_world) = ranger.ray_trace(query_pipeline, rigid_bodies, colliders, coord_sys_origin, coord_sys_angle);
        let ray_start_world = rna::point![coord_sys_origin.x + ranger.ray_local_origin.x, coord_sys_origin.y + ranger.ray_local_origin.y];
        let ray_hit = rna::Point2::new(ray_start_world.x + ray_length * ray_angle_world.cos(), ray_start_world.y + ray_length * ray_angle_world.sin());

        let kiss3d_ray_start_world: kna::OPoint<f32, kna::Const<3>> = kna::Point3::new(ray_start_world.x, ray_start_world.y, 0.);
        let kiss3d_ray_target_world: kna::OPoint<f32, kna::Const<3>> = kna::Point3::new(ray_hit.x, ray_hit.y, 0.);

        let color = kna::Point3::new(0., 0., 150.);
        scene_window.draw_line(&kiss3d_ray_start_world, &kiss3d_ray_target_world, &color);
        println!("{i}: {ray_length}");
        ranges[i] = ray_length;
    };
    ranges
}

fn simulate(behavior_policy: Policy, mut scene_window: &mut Window, cam: &mut kiss3d::camera::FirstPerson, mut rigid_bodies: RigidBodySet, mut colliders: ColliderSet, mut joint_set: ImpulseJointSet, mut body: &mut Body, body_visual: &mut RigidBodyVisual, body_radius: f32, num_frames: u32, gravity: rna::Vector2<f32>, sensor_rangers: Vec<Ranger>) -> f32 {
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
    for i in 0..num_frames {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_bodies,
            &mut colliders,
            &mut joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        scene_window.render_with_camera(cam);

        let body_rb = rigid_bodies.get_mut(body.head.handle).unwrap();
        body_rb.apply_impulse(rna::vector![0., 8. * body_radius.powf(2.)], true);
 
        set_motor_position(&mut joint_set, &mut body.legs.0.knee, -PI/6. - PI/8. * ((i as f32)*PI/30. - PI).sin());
        set_motor_position(&mut joint_set, &mut body.legs.1.knee, -PI/6. - PI/8. * ((i as f32)*PI/30. - PI/3.).sin());
        set_motor_position(&mut joint_set, &mut body.hips.0, PI/4. + PI/8. * ((i as f32)*PI/30.).sin());
        set_motor_position(&mut joint_set, &mut body.hips.1, -PI/4. - PI/8. * ((i as f32)*PI/30. + 2.*PI/3.).sin());

        cam.look_at(kna::Point3::new(body_rb.translation().x, body_rb.translation().y, 200.), kna::Point3::new(body_rb.translation().x, body_rb.translation().y, 0.));

        let body_rb_read = body_rb.clone();

        draw_body(&mut scene_window, &rigid_bodies, &mut body, body_visual);

        let left = kna::Point3::new(-1000., 0., 0.);
        let right = kna::Point3::new(1000., 0., 0.);
        let color = kna::Point3::new(0., 150., 0.);
        scene_window.draw_line(&left, &right, &color);
        
        let head_pos: nalgebra::OPoint<f32, nalgebra::Const<2>> = point![body_rb_read.translation().x, body_rb_read.translation().y];
        let head_angle = body_rb_read.rotation().angle();

        draw_sensor_rays(&mut scene_window, &query_pipeline, &rigid_bodies, &colliders, &sensor_rangers, head_pos, head_angle);

        let ray_start_world = local_to_world_position(head_pos, head_angle, sensor_rangers[3].ray_local_origin);
        let ray_angle_world = head_angle + sensor_rangers[3].ray_local_angle;
        let ray = Ray::new(ray_start_world, rna::vector![ray_angle_world.cos(), ray_angle_world.sin()]);

        let filter = QueryFilter::default();
        let mut ray_length = 0.;
        if let Some(cast_result) = query_pipeline.cast_ray(&rigid_bodies, &colliders, &ray, 10000., true, filter) {
            ray_length = cast_result.1;
        };
        let ray_hit = rna::Point2::new(ray_start_world.x + ray_length * ray_dir_world.x, ray_start_world.y + ray_length * ray_dir_world.y);

        ray_length = 

        let kiss3d_ray_start_world: kna::OPoint<f32, kna::Const<3>> = kna::Point3::new(ray_start_world.x, ray_start_world.y, 0.);
        let kiss3d_ray_target_world: kna::OPoint<f32, kna::Const<3>> = kna::Point3::new(ray_hit.x, ray_hit.y, 0.);

        let color = kna::Point3::new(0., 0., 150.);
        scene_window.draw_line(&kiss3d_ray_start_world, &kiss3d_ray_target_world, &color);

        println!("{ray_length}, {ray_start_world}, {ray_hit}");
        thread::sleep(Duration::from_millis(30));
    }

    let body_rb = rigid_bodies.get(body.head.handle).unwrap();
    body_rb.translation().x
}

impl State for StateParams {
    type A = LegRhythmAction;
    fn reward(&self) -> f64 {
        (&self.sensors.position_int.x * &self.sensors.increment_int).into()
    }
    fn actions(&self) -> Vec<LegRhythmAction> {
        vec![
            // Increments
            LegRhythmAction {hip1_amplitude: 1, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 1, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 1, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 1, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 1, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 1, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 1, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 1, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 1, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 1, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 1, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 1,
            },
            // Decrements
            LegRhythmAction {hip1_amplitude: -1, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: -1, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: -1, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: -1, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: -1, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: -1, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: -1, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: -1, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: -1, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: -1, knee2_frequency: 0, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: -1, knee2_phase: 0,
            },
            LegRhythmAction {hip1_amplitude: 0, hip1_frequency: 0, hip1_phase: 0, hip2_amplitude: 0, hip2_frequency: 0, hip2_phase: 0, knee1_amplitude: 0, knee1_frequency: 0, knee1_phase: 0, knee2_amplitude: 0, knee2_frequency: 0, knee2_phase: -1,
            },
        ]
    }
}

fn make_reward_fn(mut scene_window: &mut Window, cam: kiss3d::camera::FirstPerson, rigid_bodies: RigidBodySet, colliders: ColliderSet, joint_set: ImpulseJointSet, body: Body, body_visual: &mut RigidBodyVisual, body_radius: f32, num_frames: u32, gravity: rna::Vector2<f32>, sensor_rangers: Vec<Ranger>) -> impl FnMut(Policy) -> f32 {
    move |behavior_policy| simulate(behavior_policy, &mut scene_window, &mut cam.clone(), rigid_bodies.clone(), colliders.clone(), joint_set.clone(), &mut body.clone(), &mut body_visual.clone(), body_radius, num_frames, gravity, sensor_rangers.clone())
}
/*
impl State for LegRhythmState {
    type A = StateAction;
    fn reward(&self) -> f32 {
        simulate
    }
}
*/

fn make_cuboid_collider_and_handle(mut rigid_bodies: &mut RigidBodySet, colliders: &mut ColliderSet, pos: rna::Vector2<f32>, ang: f32, hx: f32, hy: f32) -> RigidBodyHandle {
    let body = RigidBodyBuilder::dynamic()
        .translation(pos)
        .rotation(ang)
        .build();
    let collider = ColliderBuilder::cuboid(hx, hy).restitution(0.7).build();
    let handle = rigid_bodies.insert(body);

    colliders.insert_with_parent(collider, handle, &mut rigid_bodies);

    handle
}

fn make_leg(rigid_bodies: &mut RigidBodySet, colliders: &mut ColliderSet, joint_set: &mut ImpulseJointSet, &start_pos: &rna::Vector2<f32>, &thigh_length: &f32, &calf_length: &f32, &knee_stiff: &f32, &knee_damp: &f32) -> Leg {
    let knee_ang: f32 = PI/4.;

    let thigh_pos = rna::vector![0., calf_length + thigh_length + 1.] + start_pos;
    let calf_pos = rna::vector![0., calf_length - 1.] + start_pos;

    let thigh_handle = make_cuboid_collider_and_handle(rigid_bodies, colliders, thigh_pos, -knee_ang, 1., thigh_length);
    let calf_handle = make_cuboid_collider_and_handle(rigid_bodies, colliders, calf_pos, -knee_ang, 1., calf_length);

    let calf_rb = rigid_bodies.get_mut(calf_handle).unwrap();
    calf_rb.set_additional_mass(3. *calf_rb.mass(), true);

    let knee = RevoluteJointBuilder::new()
        .local_anchor1(rna::point![0., -1.05 * thigh_length])
        .local_anchor2(rna::point![0., 1.05 * calf_length])
        .motor_position(0., knee_stiff, knee_damp);

    let knee_handle = joint_set.insert(thigh_handle, calf_handle, knee, true);

    Leg{thigh: Rod{handle: thigh_handle, length: thigh_length}, 
        calf: Rod{handle: calf_handle, length: calf_length}, 
        knee: Hinge{handle: knee_handle, stiff: knee_stiff, damp: knee_damp}}
}

fn make_body(mut rigid_bodies: &mut RigidBodySet, mut colliders: &mut ColliderSet, mut joint_set: &mut ImpulseJointSet, &start_pos: &rna::Vector2<f32>, &radius: &f32, &thigh_length: &f32,  &calf_length: &f32, &joint_stiff: &f32, &joint_damp: &f32) -> Body {
    let head_rb = RigidBodyBuilder::dynamic()
        .translation(rna::vector![start_pos.x, 2. * thigh_length + 2. * calf_length + radius])
        .build();
    let head_collider = ColliderBuilder::ball(radius).restitution(0.7).build();
    let head_handle = rigid_bodies.insert(head_rb);
    colliders.insert_with_parent(head_collider, head_handle, &mut rigid_bodies);

    let head = Head{handle: head_handle, radius: radius};

    let (leg1, leg2) = (make_leg(&mut rigid_bodies, &mut colliders, &mut joint_set, &start_pos, &thigh_length, &calf_length, &joint_stiff, &joint_damp), 
        make_leg(&mut rigid_bodies, &mut colliders, &mut joint_set, &start_pos, &thigh_length, &calf_length, &joint_stiff, &joint_damp));

    let (hip1_builder, hip2_builder) = (RevoluteJointBuilder::new()
        .local_anchor1(2.05*rna::point![radius.sqrt(), -radius.sqrt()])
        .local_anchor2(rna::point![0., radius + thigh_length/2.])
        .motor_position(PI/4., joint_stiff, joint_damp),

        RevoluteJointBuilder::new()
        .local_anchor1(2.05*rna::point![-radius.sqrt(), -radius.sqrt()])
        .local_anchor2(rna::point![0., radius + thigh_length/2.])
        .motor_position(-PI/4., joint_stiff, joint_damp));

    let (hip1_handle, hip2_handle) = (joint_set.insert(head.handle, leg1.thigh.handle, hip1_builder, true),
    joint_set.insert(head.handle, leg2.thigh.handle, hip2_builder, true));

    let (hip1, hip2) = (Hinge{handle: hip1_handle, stiff: joint_stiff, damp: joint_damp},
        Hinge{handle: hip2_handle, stiff: joint_stiff, damp: joint_damp});

    let body = Body{head: head, legs: (leg1, leg2), hips: (hip1, hip2)};
    body
}

fn draw_leg(scene_window: &mut Window, rigid_bodies: &RigidBodySet, leg: &mut Leg) {
    let thigh_rb = rigid_bodies.get(leg.thigh.handle).unwrap();
    let thigh_pos = kna::Point3::new(thigh_rb.translation().x, thigh_rb.translation().y, 0.);
    let thigh_ang = thigh_rb.rotation().to_polar().1;
    let thigh_top = kna::Point3::new(thigh_pos.x - leg.thigh.length * thigh_ang.sin(), thigh_pos.y + leg.thigh.length * thigh_ang.cos(), 0.);
    let thigh_bot = kna::Point3::new(thigh_pos.x + leg.thigh.length * thigh_ang.sin(), thigh_pos.y - leg.thigh.length * thigh_ang.cos(), 0.);

    let calf_rb = rigid_bodies.get(leg.calf.handle).unwrap();
    let calf_pos = kna::Point3::new(calf_rb.translation().x, calf_rb.translation().y, 0.);
    let calf_ang = calf_rb.rotation().to_polar().1;
    let calf_top = kna::Point3::new(calf_pos.x - leg.calf.length * calf_ang.sin(), calf_pos.y + leg.calf.length * calf_ang.cos(), 0.);
    let calf_bot = kna::Point3::new(calf_pos.x + leg.calf.length * calf_ang.sin(), calf_pos.y - leg.calf.length * calf_ang.cos(), 0.);

    let color = kna::Point3::new(150., 0., 0.);

    scene_window.draw_line(&thigh_top, &thigh_bot, &color);
    scene_window.draw_line(&calf_top, &calf_bot, &color);
}

fn make_head_visual(scene_window: &mut Window, rigid_bodies: &mut RigidBodySet, body_handle: RigidBodyHandle, body_radius: f32) -> scene::SceneNode {
    let ball_rb: &mut RigidBody = rigid_bodies.get_mut(body_handle).unwrap();
    let ball_center = kna::Translation3::new(ball_rb.translation().x, ball_rb.translation().y, 0.);

    let mut ball = scene_window.add_sphere(body_radius);
    ball.append_translation(&ball_center);

    ball
}

fn update_head_visual(rigid_bodies: &RigidBodySet, body_handle: RigidBodyHandle, mut body_visual: scene::SceneNode) -> scene::SceneNode {
    let body_rb = rigid_bodies.get(body_handle).unwrap();

    let vis_x = body_visual.data().world_transformation().translation.x;
    let vis_y = body_visual.data().world_transformation().translation.y;
    body_visual.append_translation(&kna::Translation3::new(body_rb.position().translation.x - vis_x, body_rb.position().translation.y - vis_y, 0.));

    body_visual
}

fn draw_body(scene_window: &mut Window, rigid_bodies: &RigidBodySet, body: &mut Body, visual: &mut RigidBodyVisual) {
    update_head_visual(rigid_bodies, body.head.handle, visual.node.clone());
    draw_leg(scene_window, rigid_bodies, &mut body.legs.0);
    draw_leg(scene_window, rigid_bodies, &mut body.legs.1);
}

fn main() {
    let mut rigid_bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joint_set = ImpulseJointSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(10000.0, 1.).build(); 
    colliders.insert(collider);

    /* Create the leg parts. */
    let thigh_length = 7.5;
    let calf_length = 10.;
    let start_height_offset = 15.;
    let start_pos = rna::vector![-700., calf_length + start_height_offset];
    let knee_stiff = 10000.;
    let knee_damp = 30.;

    /* Create the body. */
    let head_height_offset = 5.;
    let head_radius = 5.;

    let head_rb = RigidBodyBuilder::dynamic()
        .translation(rna::vector![start_pos.x, 2. * thigh_length + 2. * calf_length + head_height_offset + start_height_offset + head_radius])
        .build();
    let head_collider = ColliderBuilder::ball(head_radius).restitution(0.7).build();
    let head_handle = rigid_bodies.insert(head_rb.clone());
    colliders.insert_with_parent(head_collider, head_handle, &mut rigid_bodies);

    /* Create the sensors. */
    let local_front_sensor_pos = point![1.3*head_radius, 0.];
    let local_rear_sensor_pos = point![-1.3*head_radius, 0.];

    let sensor_angles = [-PI * (0.0625), -PI * (0.125), -PI * (0.25)];
    let head_angle = head_rb.rotation().angle();
    let head_pos = point![head_rb.translation().x, head_rb.translation().y];

    let mut sensor_rangers = Vec::new();

    for angle in sensor_angles {
        let front_sensor_pos: nalgebra::OPoint<f32, nalgebra::Const<2>> = local_to_world_position(head_pos, head_angle, local_front_sensor_pos);
        let rear_sensor_pos = local_to_world_position(head_pos, head_angle, local_rear_sensor_pos);
        
        let front_dir = rna::Vector2::new(angle.cos(), angle.sin());
        let rear_dir = rna::Vector2::new((PI + angle).cos(), (-PI - angle).sin());

        sensor_rangers.push(Ranger{origin: local_front_sensor_pos, dir: front_dir});
        sensor_rangers.push(Ranger{origin: local_rear_sensor_pos, dir: rear_dir});
    }

    /* Create the kiss3d window & scene. */
    let (res_x, res_y) = (512, 512);
    let mut scene_window = Window::new_with_size("scene_window", res_x, res_y);
    let mut cam = kiss3d::camera::FirstPerson::new(kna::Point3::new(0., 0., 200.), kna::Point3::new(0., 0., 0.));

    let head_node = make_head_visual(&mut scene_window, &mut rigid_bodies, head_handle, head_radius);

    let mut body = make_body(&mut rigid_bodies, &mut colliders, &mut joint_set, &start_pos, &head_radius, &thigh_length, &calf_length, &knee_stiff, &knee_damp);

    let mut head_visual = RigidBodyVisual{handle: body.head.handle, node: head_node};

    let balloonbot_state = StateParams {
        kind: StateParamsKind::BalloonBotParams,
        actuators: BalloonBotActuatorValues::new_zeros(1.),
        sensors: 
        increment: PI/20.,
    };

    let policy = Policy{kind: PolicyKind::BalloonBotPolicy, inputs: BalloonBotSensorValues::new_zeros(), outputs: ActuatorValues{vec: vec![0.]}, parameters: BalloonBotActuatorValues::new_zeros(PI/100.)};

    /* Create other structures necessary for the simulation. */
    simulate(policy, &mut scene_window, &mut cam, rigid_bodies, colliders, joint_set, &mut body, &mut head_visual, head_radius, 10000, rna::vector![0.0, -98.1], sensor_rangers);
}