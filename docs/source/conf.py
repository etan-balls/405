import os
import sys

# Make source modules importable for autodoc
sys.path.insert(0, os.path.abspath('../..'))

# ── Project info ──────────────────────────────────────────────────────────────
project   = 'Romi Autonomous Robot'
copyright = '2026, Fredy Herrarte and Ethan Liu'
author    = 'Fredy Herrarte and Ethan Liu'
release   = '1.0'

# ── Extensions ────────────────────────────────────────────────────────────────
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
    'sphinx.ext.githubpages',
    'sphinx_copybutton',
]

# autodoc settings
autodoc_member_order    = 'bysource'
autodoc_default_options = {
    'members':          True,
    'undoc-members':    True,
    'show-inheritance': True,
}

# Napoleon (Google + NumPy style docstrings)
napoleon_google_docstring = True
napoleon_numpy_docstring  = True
napoleon_use_param        = True
napoleon_use_rtype        = True

# copybutton — skip REPL prompt characters
copybutton_prompt_text      = r'>>> |\.\.\. |\$ '
copybutton_prompt_is_regexp = True

# ── Source ────────────────────────────────────────────────────────────────────
templates_path   = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# ── HTML theme — Furo ─────────────────────────────────────────────────────────
html_theme = 'furo'

html_theme_options = {
    "sidebar_hide_name":    False,
    "navigation_with_keys": True,

    "light_css_variables": {
        "color-brand-primary":         "#0ea5e9",
        "color-brand-content":         "#0ea5e9",
        "color-admonition-background": "rgba(14,165,233,0.05)",
        "font-stack":                  "Inter, system-ui, sans-serif",
        "font-stack--monospace":       "'JetBrains Mono', 'Fira Code', monospace",
    },
    "dark_css_variables": {
        "color-brand-primary":         "#38bdf8",
        "color-brand-content":         "#38bdf8",
        "color-background-primary":    "#0f172a",
        "color-background-secondary":  "#1e293b",
        "color-background-hover":      "#1e293b",
        "color-foreground-primary":    "#e2e8f0",
        "color-foreground-secondary":  "#94a3b8",
        "color-foreground-muted":      "#64748b",
        "color-admonition-background": "rgba(56,189,248,0.07)",
        "font-stack":                  "Inter, system-ui, sans-serif",
        "font-stack--monospace":       "'JetBrains Mono', 'Fira Code', monospace",
    },
}

html_title        = "Romi Robot — ME405"
html_short_title  = "Romi"
html_static_path  = ['_static']
html_css_files    = ['custom.css']

html_show_sphinx    = False
html_show_copyright = True
