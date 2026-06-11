class VehicleInfo(object):

    def __init__(self):
        """
        waf_target: option passed to waf's --target to create binary
        default_params_filename: filename of default parameters file. Taken to
        be relative to the autotest dir.
        """
        self.options = {
            "Rover": {
                "default_frame": "rover-skid",
                "frames": {
                    "rover": {
                        "waf_target": "bin/ardurover",
                        "default_params_filename": "default_params/rover.parm",
                    },
                    "rover-skid": {
                        "waf_target": "bin/ardurover",
                        "default_params_filename": [
                            "default_params/rover.parm",
                            "default_params/rover-skid.parm",
                        ],
                    },
                    "gazebo-rover": {
                        "waf_target": "bin/ardurover",
                        "default_params_filename": [
                            "default_params/rover.parm",
                            "default_params/rover-skid.parm",
                        ],
                    },
                    "airsim-rover": {
                        "waf_target": "bin/ardurover",
                        "default_params_filename": [
                            "default_params/rover.parm",
                            "default_params/airsim-rover.parm",
                        ],
                    },
                    "calibration": {
                        "extra_mavlink_cmds": "module load sitl_calibration;",
                    },
                },
            },
        }

    def default_frame(self, vehicle):
        return self.options[vehicle]["default_frame"]

    def default_waf_target(self, vehicle):
        """Returns a waf target based on vehicle type."""
        default_frame = self.default_frame(vehicle)
        return self.options[vehicle]["frames"][default_frame]["waf_target"]

    def options_for_frame(self, frame, vehicle, opts):
        """Return information about how to run SITL for a Rover frame."""
        frames = self.options[vehicle]["frames"]
        ret = frames.get(frame)
        if ret is None:
            print("WARNING: no config for frame (%s)" % frame)
            ret = {}

        if "model" not in ret:
            ret["model"] = frame

        if "sitl-port" not in ret:
            ret["sitl-port"] = True

        if opts.model is not None:
            ret["model"] = opts.model

        if "waf_target" not in ret:
            ret["waf_target"] = self.default_waf_target(vehicle)

        if opts.build_target is not None:
            ret["waf_target"] = opts.build_target

        return ret
