local configs = require("nvim-treesitter.configs")
configs.setup {
	ensure_installed = {"cpp", "lua", "javascript", "java", "typescript", "json", "tsx"  },
	sync_install = false,
	ignore_install = { "" },
	highlight = {
		enable = true,
		disable = { "" },
		additional_vim_regex_highlighting = true,
	},
	indent = { enable = true, disable = { "yaml" } },
	rainbow = {
		enable = true,
		-- disable = { "jsx", "cpp" },
		extended_mode = true,
		max_file_lines = nil,
	},
	autotag = { enable = true },
}
