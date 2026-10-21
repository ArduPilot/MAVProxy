'''Attitude ribbons use FlightCoach's geometry and colour bands.'''

import math

import pytest

vtk = pytest.importorskip('vtk')

from MAVProxy.modules.mavproxy_map3d.elements import ElementManager

HOME = (-35.363262, 149.165238, 684.0)


def elements(zexag=1.0):
    em = ElementManager(vtk.vtkRenderer(), HOME[0], HOME[1], zexag)
    return em


def fly(em, attitudes):
    for i, attitude in enumerate(attitudes):
        em.add_trail_point(HOME[0] + i * 1.0e-4, HOME[1], HOME[2],
                           attitude=tuple(map(math.radians, attitude)))


def mesh(em):
    return em.actors['trail'][0].GetMapper().GetInput()


# Reference results from FlightCoach's RibbonMesh.js with field rotation 0,
# evaluated with its bundled Three.js Euler and Quaternion implementations.
# Pitched cases distinguish pilot roll from just using the body roll angle.
@pytest.mark.parametrize('attitude,top,bottom', [
    ((0, 0, 90), 0xffbb33, 0xaa00ff),
    ((7, 0, 90), 0xffbb33, 0xaa00ff),
    ((22, 0, 90), 0xffdd33, 0xd400ff),
    ((37, 0, 90), 0xffff33, 0xff00d4),
    ((52, 0, 90), 0xddff33, 0xff00aa),
    ((67, 0, 90), 0xbbff33, 0xff0080),
    ((82, 0, 90), 0x99ff33, 0xff0055),
    ((90, 0, 90), 0x99ff33, 0xff0055),
    ((180, 0, 90), 0xffbb33, 0xaa00ff),
    ((140, 0, 90), 0xffff33, 0xff00d4),
    ((-37, 0, 90), 0xffff33, 0xff00d4),
    ((25, 40, 135), 0xffbb33, 0xaa00ff),
    ((25, 40, 0), 0xbbff33, 0xff0080),
    ((25, 40, 90), 0xffdd33, 0xd400ff),
    ((25, 40, 270), 0xffdd33, 0xd400ff),
    ((0, 90, 0), 0x99ff33, 0xff0055),
])
def test_flightcoach_face_colours(attitude, top, bottom):
    em = elements()
    em.set_trail_ribbon(True)
    fly(em, [(0, 0, 0), attitude])
    poly = mesh(em)
    assert poly.GetNumberOfPolys() == 8
    assert poly.GetNumberOfLines() == 0
    colours = poly.GetCellData().GetScalars()
    expected = [tuple(bytes.fromhex('%06x' % colour))
                for colour in (top, 0x00bb00, bottom, 0xff0000)]
    assert [colours.GetTuple3(i) for i in range(8)] == [
        colour for colour in expected for _ in range(2)]


@pytest.mark.parametrize('roll,right,up', [
    (0, (10, 0, 0), (0, 0, 10 / 15)),
    (90, (0, 0, -10), (10 / 15, 0, 0)),
    (180, (-10, 0, 0), (0, 0, -10 / 15)),
])
def test_wings_follow_the_aircraft(roll, right, up):
    em = elements()
    em.set_trail_ribbon(True, 20)
    fly(em, [(roll, 0, 0)] * 2)
    poly = mesh(em)
    for index, (wing, side) in enumerate(((-1, -1), (1, -1),
                                         (1, 1), (-1, 1))):
        expected = tuple(wing * right[j] + side * up[j] +
                         (HOME[2] if j == 2 else 0) for j in range(3))
        assert poly.GetPoint(index) == pytest.approx(expected, abs=1.0e-4)


def test_combined_attitude_and_vertical_exaggeration():
    em = elements(zexag=2.0)
    em.set_trail_ribbon(True, 20)
    fly(em, [(25, 40, 135)] * 2)
    # FlightCoach wing corners, transformed from NED to map ENU, zexag=2.
    expected = [(4.96152869, 8.25404727, 5.54919136),
                (-4.01383432, -8.40484501, -7.40058348),
                (-4.96152869, -8.25404727, -5.54919136),
                (4.01383432, 8.40484501, 7.40058348)]
    for i, (e, n, u) in enumerate(expected):
        assert mesh(em).GetPoint(i) == pytest.approx(
            (e, n, u + HOME[2] * 2), abs=1.0e-4)


def test_switching_and_resizing_preserve_the_retained_attitude():
    em = elements()
    fly(em, [(90, 0, 0)] * 2)
    assert mesh(em).GetNumberOfLines() == 1
    em.set_trail_ribbon(True, 20)
    assert mesh(em).GetNumberOfPolys() == 8
    assert mesh(em).GetBounds()[5] - mesh(em).GetBounds()[4] == 20
    em.set_trail_ribbon(True, 40)
    assert mesh(em).GetBounds()[5] - mesh(em).GetBounds()[4] == 40
    assert len(em.trail) == 2
    em.set_trail_ribbon(False, 40)
    assert mesh(em).GetNumberOfLines() == 1
    assert mesh(em).GetNumberOfPolys() == 0
    assert em.actors['trail'][0].GetProperty().GetColor() == (1, 1, 0)


@pytest.mark.parametrize('ribbon', [False, True])
def test_trail_limit_keeps_attitudes_with_their_positions(ribbon):
    em = elements()
    em.set_trail_ribbon(ribbon, 20)
    # Fill the retained history, then add one live sample across the limit.
    em.trail = [((float(i), 0, HOME[2]), (math.pi / 2, 0, 0))
                for i in range(5000)]
    em.add_trail_point(*HOME, attitude=(math.pi, 0, 0))
    assert len(em.trail) == 5000
    assert em.trail[0][0] == (1, 0, HOME[2])
    assert em.trail[0][1] == (math.pi / 2, 0, 0)
    assert em.trail[-1][1] == (math.pi, 0, 0)
    if ribbon:
        assert mesh(em).GetNumberOfPoints() == 20000
        assert mesh(em).GetNumberOfPolys() == 8 * 4999
        # The first station is banked 90 degrees; the last is inverted.
        assert mesh(em).GetPoint(1)[2] == pytest.approx(HOME[2] - 10)
        assert mesh(em).GetPoint(4 * 4999 + 1)[0] == pytest.approx(-10)
    else:
        assert mesh(em).GetNumberOfPoints() == 5000
        assert mesh(em).GetNumberOfLines() == 1
