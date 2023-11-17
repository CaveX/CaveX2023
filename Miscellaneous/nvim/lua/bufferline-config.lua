require("bufferline").setup{
    options = {
        mode = "buffers",
        separator_style = "slope",
        color_icons = true,
        diagnostics = "nvim_lsp",
        -- indicator = {
        --     style = "underline",
        -- }
        offsets = {
            { 
                filetype = "NvimTree", 
                text = "", 
                highlight = "Directory"
            },
        },
    }

}
