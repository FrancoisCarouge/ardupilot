# encoding: utf-8

"""
Adds the header-only Kalman, TypedLinearAlgebra and mp-units libraries to a
Waf build

AP_FLAKE8_CLEAN
"""


def configure(cfg):
    modules = cfg.srcnode.abspath() + '/modules/'

    for name in ('Kalman', 'TypedLinearAlgebra', 'mp-units'):
        cfg.env.append_value('GIT_SUBMODULES', name)

    cfg.env.append_value('INCLUDES', [
        modules + 'Kalman/include',
        modules + 'TypedLinearAlgebra/include',
        # mp-units integration, shipped as an example plug-in
        modules + 'TypedLinearAlgebra/support/mp_units',
    ])

    # mp-units headers are third-party system headers: ArduPilot's warning
    # flags such as -Werror=undef do not apply to them
    for path in ('mp-units/src/core/include', 'mp-units/src/systems/include'):
        cfg.env.append_value('CXXFLAGS', ['-isystem', modules + path])

    # freestanding mp-units: no exceptions, no contract checks
    cfg.env.append_value('DEFINES', [
        'MP_UNITS_HOSTED=0',
        'MP_UNITS_API_CONTRACTS=0',
    ])
