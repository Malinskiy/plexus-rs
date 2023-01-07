use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::math::Vec3A;
use bevy::prelude::*;
use bevy::window::close_on_esc;
use lazy_static::*;
use rand::{prelude::*, Rng};
use std::f32::consts::PI;
use std::ops::{Mul, Range};
use std::time::Duration;
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};
use smooth_bevy_cameras::{LookTransform, LookTransformBundle, LookTransformPlugin, Smoother};
use smooth_bevy_cameras::controllers::fps::{FpsCameraBundle, FpsCameraController, FpsCameraPlugin};

const NUM_BODIES: usize = 128;
const CONNECTION_DISTANCE: f32 = 20000.;
pub const BACKGROUND: Color = Color::rgb(0.82, 0.82, 0.82);
const VERTICE_COLOR: Color = Color::BLACK;
const EDGE_COLOR: Color = Color::BLACK;
const VERTICE_RADIUS: f32 = 3.0;
const ELLIPTIC_MOVEMENT_RANGE: Range<f32> = 3.0..15.0;
const ELLIPTIC_RATIO_RANGE: Range<f32> = 1.0..5.0;
const ELLIPTIC_MOVEMENT_PERIOD: Duration = Duration::from_millis(1500);
const VERTICE_BOX_DEPTH: Range<f32> = 0f32..25f32;
const CONNECTION_DURATION: f32 = 0.1;

fn main() {
    App::new()
        .insert_resource(ClearColor(BACKGROUND))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(Simulation {
            scale: 2e5,
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(LookTransformPlugin)
        .add_plugin(FpsCameraPlugin::default())
        .add_startup_system(setup)
        .add_system(elliptic_movement_system)
        .add_system(move_sync_system)
        .add_system(close_on_esc)
        .add_system(rotator_system)
        .add_system(local_clustering)
        // .add_plugin(FrameTimeDiagnosticsPlugin::default())
        // .add_plugin(LogDiagnosticsPlugin::default())
        .run();
}

#[derive(Clone, Debug, Default, Component)]
pub struct Body {
    position: Vec3A,
}

pub type PbrVertice = VerticeBundle<StandardMaterial>;

#[derive(Bundle, Clone)]
pub struct VerticeBundle<M: Material> {
    pub mesh: Handle<Mesh>,
    pub material: Handle<M>,
    pub transform: Transform,
    pub global_transform: GlobalTransform,
    /// User indication of whether an entity is visible
    pub visibility: Visibility,
    /// Algorithmically-computed indication of whether an entity is visible and should be extracted for rendering
    pub computed_visibility: ComputedVisibility,
    pub body: Body,
}

impl<M: Material> Default for VerticeBundle<M> {
    fn default() -> Self {
        Self {
            mesh: Default::default(),
            material: Default::default(),
            transform: Default::default(),
            global_transform: Default::default(),
            visibility: Default::default(),
            computed_visibility: Default::default(),
            body: Default::default(),
        }
    }
}

#[derive(Debug, Resource)]
struct Simulation {
    pub accumulator: f32,
    pub is_paused: bool,
    pub scale: f32,
    pub timestep: f32,
}

impl Default for Simulation {
    fn default() -> Simulation {
        Simulation {
            accumulator: 0.0,
            is_paused: false,
            scale: 5e4,
            timestep: 1. / 30.,
        }
    }
}

impl Simulation {
    fn update(&mut self, time: &Time) {
        if !self.is_paused {
            self.accumulator += time.delta_seconds();
        }
    }

    fn step(&mut self) -> Option<f32> {
        if !self.is_paused && self.accumulator > self.timestep {
            self.accumulator -= self.timestep;
            return Some(self.timestep * self.scale);
        }
        None
    }
}

#[derive(Component)]
struct Rotates;

// Parametric elliptic equation
// alpha should be orthogonal to beta
#[derive(Component)]
struct EllipticMovement {
    center: Vec3A,
    alpha: Vec3A,
    beta: Vec3A,
    phase: f32,
}

fn elliptic_movement_system(
    time: Res<Time>,
    mut query: Query<(&mut Body, &mut EllipticMovement)>,
) {
    let mut bodies = query.iter_mut().collect::<Vec<_>>();
    // dbg!(&bodies);

    for (body, movement) in bodies.iter_mut() {
        let t = time.elapsed_seconds_wrapped() + movement.phase;
        //By default this will have a period of 2 * Pi unit
        let t = t * (2. * PI) * 1000. / (ELLIPTIC_MOVEMENT_PERIOD.as_millis() as f32);
        body.position = movement.center + t.cos() * movement.alpha + t.sin() * movement.beta;
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut rng = StdRng::seed_from_u64(0);

    let mesh = meshes.add(
        Mesh::from(shape::Icosphere {
            radius: VERTICE_RADIUS,
            subdivisions: 3,
        })
    );

    for _index in 0..NUM_BODIES {
        let r = rng.gen_range(2f32..800f32);
        let theta = rng.gen_range(0f32..2.0 * PI);
        let position = Vec3A::new(
            r * f32::cos(theta),
            rng.gen_range(VERTICE_BOX_DEPTH), //Pack the vertices with small depth
            r * f32::sin(theta),
        );
        
        let elliptic_alpha = Vec3A::new(
            rng.gen_range(ELLIPTIC_MOVEMENT_RANGE), 
            rng.gen_range(ELLIPTIC_MOVEMENT_RANGE), 
            rng.gen_range(ELLIPTIC_MOVEMENT_RANGE)
        );
        let elliptic_ratio = rng.gen_range(ELLIPTIC_RATIO_RANGE);
        let elliptic_beta = elliptic_alpha.any_orthogonal_vector().normalize() * elliptic_alpha.length() * elliptic_ratio; 
        
        commands.spawn((
            PbrVertice {
                mesh: mesh.clone(),
                material: materials.add(StandardMaterial {
                    emissive: VERTICE_COLOR,
                    alpha_mode: AlphaMode::Opaque,
                    perceptual_roughness: 1.0,
                    ..default()
                }),
                transform: Transform::default(),
                body: Body {
                    position,
                    ..Default::default()
                },
                ..default()
            },
            EllipticMovement {
                center: position,
                alpha: elliptic_alpha,
                beta: elliptic_beta,
                phase: rng.gen_range(0.0..=2.0*PI)
            }
        ));
    }

    commands
        .spawn(Camera3dBundle::default())
        .insert(FpsCameraBundle::new(
            FpsCameraController {
                translate_sensitivity: 1000.0,
                ..default()
            },
            Vec3::new(0.0, 1500.0, 0.0),
            Vec3::new(0., 0., 0.),
        ));
}

fn move_sync_system(mut query: Query<(&mut Body, &mut Transform)>) {
    for (body, mut transform) in query.iter_mut() {
        transform.translation = body.position.into();
    }
}

fn rotator_system(time: Res<Time>, mut query: Query<&mut Transform, With<Rotates>>) {
    for mut transform in query.iter_mut() {
        let t = time.elapsed_seconds();
        let r = 1100.0;
        let speed = 0.3;
        *transform = Transform::from_xyz(
            r * f32::cos(t * speed),
            (t * speed).sin() * 2000.0,
            r * f32::sin(t * speed),
        )
            .looking_at(Vec3::ZERO, Vec3::Y);
    }
}

fn local_clustering(mut query: Query<(&GlobalTransform, With<Body>)>, mut lines: ResMut<DebugLines>) {
    let mut iter = query.iter_combinations_mut();
    while let Some([(transform1, mut body1), (transform2, mut body2)]) =
        iter.fetch_next()
    {
        let delta = transform2.translation() - transform1.translation();
        let distance_sq: f32 = delta.length_squared();

        if distance_sq < CONNECTION_DISTANCE {
            let connection_coefficient = 1.0 - (distance_sq / CONNECTION_DISTANCE);
            let color_vec = (Vec4::from(EDGE_COLOR) - Vec4::from(BACKGROUND)) * connection_coefficient + Vec4::from(BACKGROUND);
            let color = Color::from(color_vec);
            lines.line_colored(transform1.translation(), transform2.translation(), CONNECTION_DURATION, color);
        }
    }
}

lazy_static! {
    static ref W0: f32 = -2f32.cbrt() / (2f32 - 2f32.cbrt());
    static ref W1: f32 = 1f32 / (2f32 - 2f32.cbrt());
    static ref C1: f32 = *W1 / 2f32;
    static ref C2: f32 = (*W0 + *W1) / 2f32;
    static ref C3: f32 = *C2;
    static ref C4: f32 = *C1;
    static ref CS: [f32; 4] = [*C1, *C2, *C3, *C4];
    static ref D1: f32 = *W1;
    static ref D2: f32 = *W0;
    static ref D3: f32 = *D1;
    static ref DS: [f32; 3] = [*D1, *D2, *D3];
}
