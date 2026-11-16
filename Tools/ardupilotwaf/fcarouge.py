# encoding: utf-8

"""
Adds the header-only Kalman, TypedLinearAlgebra, SparseLinearAlgebra, and
mp-units libraries to a Waf build

AP_FLAKE8_CLEAN
"""


def configure(cfg):
    modules = cfg.srcnode.abspath() + '/modules/'

    for name in ('Kalman', 'TypedLinearAlgebra', 'SparseLinearAlgebra', 'mp-units'):
        cfg.env.append_value('GIT_SUBMODULES', name)

    # the FrancoisCarouge libraries build under ArduPilot's own warnings
    cfg.env.append_value('INCLUDES', [
        modules + 'Kalman/include',
        modules + 'TypedLinearAlgebra/include',
        modules + 'SparseLinearAlgebra/include',
    ])

    # mp-units headers are third-party system headers: ArduPilot's warning
    # flags such as -Werror=undef do not apply to them
    for path in ('mp-units/src/core/include', 'mp-units/src/systems/include'):
        cfg.env.append_value('CXXFLAGS', ['-isystem', modules + path])

    # freestanding mp-units: no exceptions, no contract checks, no formatting
    cfg.env.append_value('DEFINES', [
        'MP_UNITS_HOSTED=0',
        'MP_UNITS_API_CONTRACTS=0',
        'MP_UNITS_API_THROWING_CONSTRAINTS=0',
    ])
