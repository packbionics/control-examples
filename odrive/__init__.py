import odrive_control as odc


def main():
    instance1 = odc.odriveControl()
    # instance1.calibration_sequence()
    instance1.torque(0.09, 2)
    instance1.torque(-0.09, 2)

    # instance1.velocity(4, 4)
    # instance1.tune_current_control_bandwidth()
    instance1.print_tunable_params()


if __name__ == "__main__":
    main()
