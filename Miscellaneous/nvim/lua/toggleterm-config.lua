require("toggleterm").setup{
	size = 15,
	open_mapping = [[<c-\>]],
	shade_filetypes = {},
	shade_terminals = true,
	shading_factor = 1,
	start_in_insert = true,
	insert_mappings = true,
	persist_size = true,
	direction = 'horizontal',
	close_on_exit = true,
	-- shell = 'powershell',
	float_opts = {
		border = 'curved',
		winblend = 3,
		highlights = {
			border = "Normal",
			background = "Normal",
		}
	}
}
