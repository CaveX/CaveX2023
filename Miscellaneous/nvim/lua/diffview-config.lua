require("diffview").setup({
	diff_binaries = false,    -- Show diffs for binaries
	use_icons = true,         -- Requires nvim-web-devicons
	watch_index = true,	   -- Update diffs if index is changed
  
	file_panel = {
		win_config = {
			position = "left", -- One of 'left', 'right', 'top', 'bottom'
			width = 35, -- Only applies when position is 'left' or 'right'
		},
	},
})
