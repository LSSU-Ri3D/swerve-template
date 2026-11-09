# How we write robot code

**The stack:** Java 17, WPILib 2026 command-based, roboRIO. Phoenix 6 for Krakens/CANcoders,
Phoenix 5 only for the Pigeon 1.0, REVLib for NEOs, Studica for the NavX, PathPlanner for autos,
Epilogue for logging, Elastic for the dashboard. Don't add another vendordep unless you actually
need it (or just ask me (ada))

## Where stuff goes

```
frc/robot/
  Main.java              don't touch
  Robot.java             runs the scheduler
  RobotContainer.java    makes every subsystem, binds every button, sets every default command
  Ports.java             every CAN ID and controller port
  commands/
    Autos.java           every auto + PathPlanner named commands
    Superstructure.java  commands that use more than one subsystem (I'll make it if we need it)
  subsystems/
    drive/               Drive, SwerveModule, Gyro, DriveConstants
    intake/              one folder per mechanism
      Intake.java
  util/                  small helpers that aren't a subsystem
```

- One folder per subsystem, named after the mechanism (`intake/`, `shooter/`, `climber/`), not
  after the motor or after whoever's writing it.
- Constants go at the top of the subsystem that uses them. A subsystem only gets its own
  `XxxConstants.java` when there's too many to read at a glance (drive has one).
- **No `Constants.java`.** It always turns into a 600 line file nobody can find anything in.
- CAN IDs are the exception, they all go in `Ports.java` so collisions are obvious.

## What a subsystem looks like

Copy `subsystems/example/` and rename it. The rules:

1. **Hardware is `private final`.** Nothing outside the subsystem touches a motor
2. **The only way to make it move is a command** from a factory method on the subsystem:
   `intake()`, `eject()`, `goTo(Setpoint.SCORE)`. No public `setSpeed()` / `setVoltage()`.
   (Exception: callbacks a library needs, like `Drive.resetPose` for PathPlanner. Comment why.)
3. **Anything other code needs to know is a `Trigger` or a getter.** `hasPiece`, `atGoal`,
   `getAngleDegrees()`.
4. **`periodic()` reads sensors and updates state, it doesn't decide anything.** If a motor is
   being set in `periodic()`, it should be a command.
5. **Every subsystem has a default command that makes it safe** (stop, or hold at stow). Default
   commands get set in `RobotContainer`, not in the subsystem.
6. **Configure every motor completely in code** and run it through `Hardware.configure(...)`.
   Never rely on something set in Phoenix Tuner or the REV Hardware Client except the CAN ID.
   Every motor gets a current limit, every one, ALEX!!!!!!
7. Position commands (arms, elevators) **hold forever and never finish**. Wait on `atGoal` instead
   of making them end, otherwise the default command grabs it and the arm falls.

## Where commands go

| What it does | Where it goes |
|---|---|
| One subsystem | factory method on that subsystem: `intake.intake()` |
| More than one subsystem | static method in `commands/Superstructure.java` |
| An auto | static method in `commands/Autos.java`, or a PathPlanner auto |
| Button → command | `RobotContainer.configureDriverBindings()` / `configureOperatorBindings()` |

- Build commands out of `run`, `runOnce`, `startEnd`, `until`, `andThen`, `alongWith`, etc.
  **Don't write a `class Foo extends Command`** unless it has real state that won't fit in a
  lambda. It's always more code and it's harder to read at 3am.
- Every factory ends with `.withName("Subsystem.verb")` so the scheduler log actually says what's
  running.
- Something PathPlanner calls by name gets registered in `Autos.registerNamedCommands` with the
  exact same string as in the GUI. Copy paste it, typos just silently skip the command.

## Naming

| Thing | Style | Example |
|---|---|---|
| Classes | PascalCase noun | `Intake`, `ShooterConstants` |
| Command factories | camelCase verb, no "Command" on the end | `intake()`, `zeroHeading()`, `goTo(...)` |
| Triggers / boolean state | reads like a yes/no | `hasPiece`, `atGoal`, `isStalled()` |
| Getters | `getX()` with the unit in the name | `getAngleDegrees()`, `getCurrentAmps()` |
| Constants | `UPPER_SNAKE` **with the unit in the name** | `MAX_SPEED_MPS`, `STOW_DEGREES`, `GEAR_RATIO` |
| CAN IDs | `MECHANISM_THING` in `Ports` | `INTAKE_ROLLER`, `FL_STEER` |

Units: SI (meters, radians, seconds) anywhere one subsystem talks to another. Degrees are fine
inside a mechanism if they're in the name. Rotations only show up inside motor configs, since
that's what Phoenix wants.

## Controls

Two Xbox controllers. Driver on port 0, operator on port 1, check the DS USB tab before every
match. The full map is in the comment above `configureDriverBindings()`, update it when you
change a binding. The driver controller is drive-only, mechanisms go on the operator unless the
strategy says otherwise.

## Logging

- Classes are `@Logged(strategy = OPT_IN)`, then put `@Logged` on the getters you want to see.
  Opt-in so we don't spam the log with stuff nobody reads.
- Everything goes to NetworkTables (Elastic) and into a `.wpilog` on the RIO automatically.
  Open those in AdvantageScope

## Git

- tbh we don't have enough time for me to teach everyone git, so fork or use worktrees