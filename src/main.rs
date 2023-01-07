use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::math::Vec3A;
use bevy::prelude::*;
use bevy::window::close_on_esc;
use lazy_static::*;
use rand::{prelude::*, Rng};
use std::f32::consts::PI;
const NUM_BODIES: usize = 128;
const BACKGROUND: Color = Color::SILVER;

fn main() {
    App::new()
        .insert_resource(ClearColor(BACKGROUND))
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(Simulation {
            scale: 1e5,
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup)
        .add_system(nbody_system)
        .add_system(move_sync_system.after(nbody_system))
        .add_system(close_on_esc)
        .add_system(rotator_system)
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_plugin(LogDiagnosticsPlugin::default())
        .run();
}

#[derive(Clone, Debug, Default, Component)]
pub struct Body {
    mass: f32,
    acceleration: Vec3A,
    velocity: Vec3A,
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

/// this component indicates what entities should rotate
#[derive(Component)]
struct Rotates;

#[derive(Component)]
struct Moves;

const G: f32 = 6.674_30E-11;
const EPSILON: f32 = 1.;

fn nbody_system(
    time: Res<Time>,
    mut simulation: ResMut<Simulation>,
    mut query: Query<(Entity, &mut Body)>,
) {
    let mut bodies = query.iter_mut().collect::<Vec<_>>();
    // dbg!(&bodies);

    // Step simulation in fixed increments
    simulation.update(&*time);
    while let Some(dt) = simulation.step() {
        // Start substeps
        for substep in 0..3 {
            // Clear accelerations and update positions
            for (_, body) in bodies.iter_mut() {
                body.acceleration = Vec3A::ZERO;
                let dx = (*CS)[substep] * body.velocity * dt;
                body.position += dx;
            }

            // Update accelerations
            for index1 in 0..bodies.len() {
                let (bodies1, bodies2) = bodies.split_at_mut(index1 + 1);
                let (_, body1) = &mut bodies1[index1];
                for (_, body2) in bodies2.iter_mut() {
                    let offset = body2.position - body1.position;
                    let distance_squared = offset.length_squared();
                    let normalized_offset = offset / distance_squared.sqrt();

                    let da = (G * body2.mass / (distance_squared + EPSILON)) * normalized_offset;
                    body1.acceleration += da;
                    body2.acceleration -= da;
                }
            }

            // Update velocities
            for (_, body) in bodies.iter_mut() {
                let dv = (*DS)[substep] * body.acceleration * dt;
                body.velocity += dv;
                if substep == 2 {
                    let dx = *C4 * body.velocity * dt;
                    body.position += dx;
                }
            }
        }
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
            radius: 2.0,
            subdivisions: 3,
        }
        )
    );

    let color_range = 0.3..=0.4;

    for _index in 0..NUM_BODIES {
        let r = rng.gen_range(2f32..800f32);
        let theta = rng.gen_range(0f32..2.0 * PI);
        let position = Vec3A::new(
            r * f32::cos(theta),
            rng.gen_range(-500f32..500f32),
            r * f32::sin(theta),
        );
        let size = rng.gen_range(50f32..1000f32);
        let greyscale_shade = rng.gen_range(color_range.clone());
        commands.spawn((
            PbrVertice {
                mesh: mesh.clone(),
                material: materials.add(StandardMaterial {
                    base_color: Color::rgb(greyscale_shade,greyscale_shade,greyscale_shade),
                    perceptual_roughness: 1.0,
                    ..default()
                }),
                transform: Transform::default(),
                body: Body {
                    mass: size,
                    position,
                    velocity: position.cross(Vec3A::Y).normalize() * 0.00019,
                    ..Default::default()
                },
                ..default()
            },
            Moves,
        ));
    }

    // camera
    commands.spawn((
        Camera3dBundle {
            camera: Camera {
                hdr: true,
                ..default()
            },
            ..default()
        },
        bevy::core_pipeline::bloom::BloomSettings {
            intensity: 0.1,
            ..default()
        },
        Rotates,
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
        *transform = Transform::from_xyz(
            r * f32::cos(t * 0.1),
            (t * 0.1).sin() * 2000.0,
            r * f32::sin(t * 0.1),
        )
        .looking_at(Vec3::ZERO, Vec3::Y);
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
