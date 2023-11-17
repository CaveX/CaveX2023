require("nvim-tree").setup({
    hijack_cursor = true, -- force cursor to always be at first character of line in nvim file tree
    filters = {
        dotfiles = true,
    },
    view = {
        centralize_selection = true,
        width = 40,
    },
    renderer = {
        highlight_diagnostics = true,
        root_folder_label = "..",
        indent_markers = {
            enable = true,
        },
        icons = {
            symlink_arrow = "  ",
            web_devicons = {
                folder = {
                    enable = true,
                }
            },
            glyphs = {
                folder = {
                    default = "󰉋",
                    open = "󰝰",
                    empty = "󰉖",
                    empty_open = "󰷏",
                }
            },
        },
    },
    live_filter = {
        prefix = " : "
    }
})
