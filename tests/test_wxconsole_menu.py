"""Console menu updates must not replace a native popup during navigation."""
from types import SimpleNamespace
from unittest import mock

import pytest

pytest.importorskip('wx')
from MAVProxy.modules.lib import wx_processguard  # noqa: E402,F401
from MAVProxy.modules.lib import wxconsole_ui  # noqa: E402


@pytest.fixture
def console(monkeypatch):
    callbacks = []
    monkeypatch.setattr(wxconsole_ui.wx, 'PlatformInfo', ('wxGTK',))
    monkeypatch.setattr(wxconsole_ui.wx, 'CallAfter',
                        lambda callback, *args: callbacks.append((callback, args)))
    old = mock.Mock()
    native = mock.Mock()
    state = SimpleNamespace(menu_open=False, menu_generation=0, menu_actions=0,
                            pending_menu=None, menu=old,
                            state=SimpleNamespace(child_pipe_send=mock.Mock()),
                            GetMenuBar=mock.Mock(return_value=native),
                            SetMenuBar=mock.Mock(), Refresh=mock.Mock(), Update=mock.Mock(),
                            IsBeingDeleted=mock.Mock(return_value=False))
    for name in ('on_menu_open', 'on_menu_close', 'finish_menu_close', 'is_top_menu',
                 'apply_pending_menu', 'on_menu'):
        setattr(state, name, lambda *args, name=name:
                getattr(wxconsole_ui.ConsoleFrame, name)(state, *args))

    def flush():
        while callbacks:
            callback, args = callbacks.pop(0)
            callback(*args)

    state.flush = flush
    state.old_native = native
    return state


def event(parent=None, menu=None):
    if menu is None:
        menu = SimpleNamespace(GetParent=lambda: parent)
    return SimpleNamespace(GetMenu=lambda: menu, Skip=mock.Mock())


def test_submenu_close_keeps_parent_menu_active(console):
    root = event()
    child = event(parent=root.GetMenu())
    console.on_menu_open(root)
    console.on_menu_open(child)
    original = console.menu
    console.pending_menu = mock.Mock()
    console.on_menu_close(child)
    console.flush()
    console.apply_pending_menu()  # another update delivered by the pipe timer
    console.SetMenuBar.assert_not_called()
    assert console.menu is original
    assert console.menu_open
    console.on_menu_close(root)
    console.flush()
    console.SetMenuBar.assert_called_once()


def test_macos_hidden_root_menu_is_not_a_parent_popup(console, monkeypatch):
    monkeypatch.setattr(wxconsole_ui.wx, 'PlatformInfo', ('wxMac',))
    hidden = SimpleNamespace(GetParent=lambda: None)
    root = event(parent=hidden)
    child = event(parent=root.GetMenu())
    console.on_menu_open(root)
    console.pending_menu = mock.Mock()
    console.on_menu_close(child)
    console.flush()
    console.SetMenuBar.assert_not_called()
    console.on_menu_close(root)
    console.flush()
    console.SetMenuBar.assert_called_once()
    assert not console.menu_open


def test_root_close_waits_for_native_handler_to_return(console):
    root = event()
    console.on_menu_open(root)
    console.pending_menu = mock.Mock()
    console.on_menu_close(root)
    console.apply_pending_menu()  # a timer during native popup cleanup
    console.SetMenuBar.assert_not_called()
    console.flush()
    console.SetMenuBar.assert_called_once()
    console.old_native.Destroy.assert_called_once()


def test_switching_top_level_menus_keeps_updates_deferred(console):
    first, second = event(), event()
    console.on_menu_open(first)
    console.pending_menu = mock.Mock()
    console.on_menu_close(first)
    console.on_menu_open(second)
    console.flush()  # first popup's deferred close must not close the second
    console.SetMenuBar.assert_not_called()
    assert console.menu_open
    console.on_menu_close(second)
    console.flush()
    console.SetMenuBar.assert_called_once()


def test_selection_after_close_uses_original_menu(console):
    root = event()
    console.on_menu_open(root)
    original = console.menu
    selected = original.find_selected.return_value
    console.pending_menu = mock.Mock()
    console.on_menu_close(root)
    selection = mock.Mock()
    console.on_menu(selection)
    original.find_selected.assert_called_once_with(selection)
    selected.call_handler.assert_called_once()
    console.state.child_pipe_send.send.assert_called_once_with(selected)
    console.flush()
    console.SetMenuBar.assert_called_once()


def test_only_latest_description_is_installed_after_close(console):
    root = event()
    console.on_menu_open(root)
    console.pending_menu = mock.Mock()
    console.on_menu_close(root)
    newest = mock.Mock()
    console.pending_menu = newest
    console.flush()
    assert console.menu is newest
    console.SetMenuBar.assert_called_once_with(newest.wx_menu.return_value)
    assert console.pending_menu is None


def test_modal_menu_handler_keeps_old_native_menu_alive(console):
    root = event()
    console.on_menu_open(root)
    selected = console.menu.find_selected.return_value
    console.pending_menu = mock.Mock()
    console.on_menu_close(root)

    def modal_handler():
        # A file dialog runs a nested event loop inside the native menu action.
        console.flush()
        console.apply_pending_menu()
        console.SetMenuBar.assert_not_called()
        console.old_native.Destroy.assert_not_called()

    selected.call_handler.side_effect = modal_handler
    console.on_menu(mock.Mock())
    console.SetMenuBar.assert_not_called()
    console.flush()
    console.SetMenuBar.assert_called_once()
    console.old_native.Destroy.assert_called_once()


def test_close_callback_after_window_shutdown_does_nothing(console):
    root = event()
    console.on_menu_open(root)
    console.pending_menu = mock.Mock()
    console.on_menu_close(root)
    console.IsBeingDeleted.return_value = True
    console.flush()
    console.SetMenuBar.assert_not_called()


def test_platform_menu_events_without_menu_object(console):
    native = SimpleNamespace(GetMenu=lambda: None, Skip=mock.Mock())
    console.on_menu_open(native)
    console.pending_menu = mock.Mock()
    console.on_menu_close(native)
    console.flush()
    console.SetMenuBar.assert_called_once()
