package frc.robot.sds;

@FunctionalInterface
public interface AbsoluteEncoderFactory<Configuration> {
    AbsoluteEncoder create(Configuration configuration);
}
