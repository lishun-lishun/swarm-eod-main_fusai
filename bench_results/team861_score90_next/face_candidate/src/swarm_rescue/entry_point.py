"""Resolve the drone class a submission's ``my_drone_eval.py`` provides.

A submission ships one module named ``my_drone_eval.py`` and may expose its
drone class in either of the two documented ways:

* a factory::

      def drone_class_for_mode(mode: str) -> type

* a class named ``MyDroneEval``

The factory wins when both are present. Participants are told this in
``doc/participants/03-提交检查与打包.md`` and ``check_submission.sh`` accepts
either form, so the evaluator has to resolve either form too -- a submission
whose entry check passed must never fail at run time merely because it used the
class form.

The module lives outside ``solutions/`` on purpose: that directory is the
contestant-owned surface (it is what ``create_zip_submission.sh`` packs into a
submission), so engine code must not sit there.

This module stays import-light: ``launcher.py`` imports it at module scope,
before any simulation module loads.
"""

from typing import Any

ENTRY_MODULE = "my_drone_eval"
FACTORY_NAME = "drone_class_for_mode"
CLASS_NAME = "MyDroneEval"


def resolve_drone_class(module: Any, mode: str) -> type:
    """Return the drone class *module* provides for *mode*.

    *module* is the imported ``swarm_rescue.solutions.my_drone_eval`` module.
    Raises ``AttributeError`` when it defines neither entry point -- which the
    evaluator already refuses at install time, so reaching it means the
    installed entry module is not the submission's.
    """
    factory = getattr(module, FACTORY_NAME, None)
    if callable(factory):
        drone_class = factory(mode)
        if drone_class is None:
            raise TypeError(
                f"{ENTRY_MODULE}.{FACTORY_NAME}({mode!r}) returned None; it must "
                "return the drone class for that mode"
            )
        return drone_class

    drone_class = getattr(module, CLASS_NAME, None)
    if isinstance(drone_class, type):
        return drone_class

    raise AttributeError(
        f"{ENTRY_MODULE}.py defines neither {FACTORY_NAME}(mode) nor "
        f"class {CLASS_NAME}"
    )


def drone_class_for_mode(mode: str) -> type:
    """Resolve the class from the installed entry module.

    Convenience wrapper for callers that only have a mode string (the launcher
    and the organizer scripts). It imports lazily so importing this module never
    pulls in participant code.
    """
    from swarm_rescue.solutions import my_drone_eval as entry_module

    return resolve_drone_class(entry_module, mode)
