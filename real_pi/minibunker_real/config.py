#!/usr/bin/env python3
# ============================================================================
#  config.py — tiny dotted-key wrapper over the YAML config, so the lifted
#  sim logic can keep its `get("behavior/limits/max_linear", default)` style
#  instead of rospy.get_param.
# ============================================================================
from __future__ import annotations

import os

import yaml

_DEFAULT = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.yaml")


class Config:
    def __init__(self, path=None):
        self.path = path or _DEFAULT
        with open(self.path, "r") as fh:
            self.data = yaml.safe_load(fh) or {}

    def get(self, dotted, default=None):
        node = self.data
        for k in str(dotted).split("/"):
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    def block(self, dotted, default=None):
        """Return a whole sub-dict (e.g. 'detector', 'behavior', 'can')."""
        val = self.get(dotted, default)
        return val if val is not None else (default or {})
