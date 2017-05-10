# compiled_pyke_files.py

from pyke import target_pkg

pyke_version = '1.1.1'
compiler_version = 1
target_pkg_version = 1

try:
    loader = __loader__
except NameError:
    loader = None

def get_target_pkg():
    return target_pkg.target_pkg(__name__, __file__, pyke_version, loader, {
         ('atc', '', 'preflight.krb'):
           [1336115090.291798, 'preflight_bc.py'],
         ('atc', '', 'pyke_demo.krb'):
           [1336113357.81045, 'pyke_demo_bc.py'],
         ('atc', '', 'questions.kqb'):
           [1336114966.799482, 'questions.qbc'],
        },
        compiler_version)

