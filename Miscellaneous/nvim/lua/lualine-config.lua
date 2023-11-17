require("lualine").setup{
    options = {
        theme = "auto",
        -- section_separators = {"", ""},
        -- component_separators = {"", ""},
        icons_enabled = true,
        disabled_filetypes = { "NvimTree" },
    },
    sections = {
        lualine_a = {"mode"},
        lualine_b = {"branch"},
        lualine_c = {"filename"},
        lualine_x = {"encoding", "fileformat", "filetype"},
        lualine_y = {"progress"},
        lualine_z = {"location", "selectioncount", "searchcount" }
    },
    inactive_sections = {
        lualine_a = {},
        lualine_b = {},
        lualine_c = {},
        lualine_x = {},
        lualine_y = {},
        lualine_z = {}
    },
    tabline = {},
    extensions = {"nvim-tree","toggleterm","trouble","nvim-dap-ui"},

}

