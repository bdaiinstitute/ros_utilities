# Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute.  All rights reserved.

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "synchros2"
copyright = "2025, Robotics and AI Institute LLC dba RAI Institute"  # noqa
author = "Robotics and AI Institute"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
    ".markdown": "markdown",
}
master_doc = "index"

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.doctest",
    "sphinx.ext.githubpages",
    "sphinx.ext.ifconfig",
    "sphinx.ext.intersphinx",
    "sphinx.ext.mathjax",
    "sphinx.ext.napoleon",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx_rtd_theme",
    "sphinxcontrib.mermaid",
]

autodoc_default_options = {
    "special-members": "__init__",
    "class-doc-from": "class",
}
autodoc_class_signature = "separated"
autodoc_mock_imports = [
    "geometry_msgs",
    "launch",
    "message_filters",
    "rclpy",
    "tf2_ros",
    "tf2_msgs",
]
intersphinx_mapping = {"python": ("https://docs.python.org/3", None)}

myst_heading_anchors = 6
pygments_style = "sphinx"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"

html_theme_options = {
    # Toc options
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
    "titles_only": False,
}

rosdoc2_settings = {
    "allow_other_extensions": True,
}
