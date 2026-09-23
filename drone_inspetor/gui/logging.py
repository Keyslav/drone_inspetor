"""Logging da GUI sem dependência do middleware ROS."""

import logging


def gui_log_info(module_name, message):
    """Registra uma atualização operacional."""
    logging.getLogger(module_name).info(message)


def gui_log_warn(module_name, message):
    """Registra uma situação recuperável."""
    logging.getLogger(module_name).warning(message)


def gui_log_error(module_name, message):
    """Registra uma falha da interface."""
    logging.getLogger(module_name).error(message)


def gui_log_debug(module_name, message):
    """Registra detalhes somente quando DEBUG está habilitado."""
    logging.getLogger(module_name).debug(message)
