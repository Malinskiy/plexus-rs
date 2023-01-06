use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(PlexusPlugin)
        .add_system(bevy::window::close_on_esc)
        .run();
}

pub struct PlexusPlugin;

#[derive(Component)]
struct Point;

impl Plugin for PlexusPlugin {
    fn build(&self, app: &mut App) {
        app.add_startup_system(add_mesh)
            .add_system(hello_world)
            .add_system(greet_people);
    }
}


fn hello_world() {
    println!("hello world!");
}

fn add_mesh(mut commands: Commands) {
    commands.spawn((Person, Name("Elaina Proctor".to_string())));
    commands.spawn((Person, Name("Renzo Hume".to_string())));
    commands.spawn((Person, Name("Zayna Nieves".to_string())));
}

fn greet_people(query: Query<&Name, With<Person>>) {
    for name in query.iter() {
        println!("hello {}!", name.0);
    }
}

#[derive(Component)]
struct Person;

#[derive(Component)]
struct Name(String);

