lua vim.loader.enable() -- makes neovim load roughly 2x faster
:set number 
:set relativenumber
:set autoindent
:set smartindent
:set tabstop=4
:set shiftwidth=4
:set smarttab
:set softtabstop=4
:set mouse=a
:set expandtab " use spaces instead of tabs for indentation
:set termguicolors
:set cursorline
:set noswapfile

:let mapleader = ','

call plug#begin()

Plug 'https://github.com/tpope/vim-surround'
Plug 'https://github.com/ryanoasis/vim-devicons'
Plug 'https://github.com/preservim/tagbar'
Plug 'lewis6991/gitsigns.nvim'
Plug 'akinsho/bufferline.nvim'
Plug 'nvim-tree/nvim-web-devicons'
Plug 'folke/tokyonight.nvim'
Plug 'nvim-treesitter/nvim-treesitter', {'do': ':TSUpdate'}
Plug 'lukas-reineke/indent-blankline.nvim'
Plug 'https://github.com/windwp/nvim-autopairs'
Plug 'https://github.com/windwp/nvim-ts-autotag'
Plug 'nvim-lua/plenary.nvim'
Plug 'nvim-telescope/telescope.nvim', {'tag': '0.1.3'}
Plug 'akinsho/toggleterm.nvim', {'tag': '*'}
Plug 'williamboman/mason.nvim'
Plug 'williamboman/mason-lspconfig.nvim'
Plug 'https://github.com/neovim/nvim-lspconfig'
Plug 'hrsh7th/nvim-cmp'
Plug 'hrsh7th/cmp-nvim-lsp'
Plug 'hrsh7th/cmp-buffer'
Plug 'hrsh7th/cmp-path'
Plug 'hrsh7th/cmp-cmdline'
Plug 'L3MON4D3/LuaSnip'
Plug 'goolord/alpha-nvim'
Plug 'sindrets/diffview.nvim'
Plug 'nvim-treesitter/nvim-treesitter-context'
Plug 'norcalli/nvim-colorizer.lua'
Plug 'onsails/lspkind-nvim'
Plug 'folke/trouble.nvim'
Plug 'jedrzejboczar/possession.nvim'
Plug 'mfussenegger/nvim-dap'
Plug 'rcarriga/nvim-dap-ui'
Plug 'theHamsta/nvim-dap-virtual-text'
Plug 'numToStr/Comment.nvim'
Plug 'Pocco81/true-zen.nvim'
Plug 'nvim-tree/nvim-tree.lua'
Plug 'nvim-lualine/lualine.nvim'

set encoding=UTF-8

call plug#end()

colorscheme tokyonight-moon

nnoremap <leader>nf :NvimTreeOpen<CR>
nnoremap <leader>nt :NvimTreeToggle<CR>
nnoremap <leader>nn :NvimTreeFindFile<CR>
nnoremap <leader>na :NvimTreeCollapseKeepBuffer<CR>

inoremap <expr> <Tab> pumvisible() ? coc#_select_confirm() : "<Tab>"

nmap <F8> :TagbarToggle<CR>

let g:loaded_netrw = 1 
let g:loaded_netrwPlugin = 1 

" nvim terminal mappings
tnoremap <Esc> <C-\><C-n><CR>

" create terminal tab
nnoremap <C-A-t> :tabnew <bar> term<CR>

" Bufferline mappings
"" Move to previous/next
nnoremap <silent> <A-,> :BufferLineCyclePrev<CR>
nnoremap <silent> <A-.> :BufferLineCycleNext<CR>

"" Re-order to previous/next
nnoremap <silent> <A-<> :BufferLineMovePrev<CR>
nnoremap <silent> <A->> :BufferLineMoveNext<CR>

"" Go to buffer in position 1-10
nnoremap <silent> <A-1> :BufferLineGoToBuffer 1<CR>
nnoremap <silent> <A-2> :BufferLineGoToBuffer 2<CR>
nnoremap <silent> <A-3> :BufferLineGoToBuffer 3<CR>
nnoremap <silent> <A-4> :BufferLineGoToBuffer 4<CR>
nnoremap <silent> <A-5> :BufferLineGoToBuffer 5<CR>
nnoremap <silent> <A-6> :BufferLineGoToBuffer 6<CR>
nnoremap <silent> <A-7> :BufferLineGoToBuffer 7<CR>
nnoremap <silent> <A-8> :BufferLineGoToBuffer 8<CR>
nnoremap <silent> <A-9> :BufferLineGoToBuffer 9<CR>

"" Pin/unpin buffer
nnoremap <silent> <A-p> :BufferLineTogglePin<CR>

"" Close buffer
nnoremap <silent> <A-w> :bd<CR>


"" START: Telescope config
nnoremap <leader>ff :Telescope find_files<CR>
nnoremap <leader>fg :Telescope live_grep<CR>
nnoremap <leader>fb :Telescope buffers<CR>
nnoremap <leader>fh :Telescope help_tags<CR>
nnoremap <leader>fo <cmd>lua require('telescope.builtin').oldfiles()<CR>
nnoremap <leader>fw <cmd>lua require('telescope.builtin').grep_string({ search = vim.fn.expand("<cword>") })<CR>
"" END: Telescope config

"" START: trouble.nvim config
nnoremap <leader>xx <cmd>TroubleToggle<cr>
nnoremap <leader>xw <cmd>TroubleToggle workspace_diagnostics<cr>
nnoremap <leader>xd <cmd>TroubleToggle document_diagnostics<cr>
nnoremap <leader>xl <cmd>TroubleToggle loclist<cr>
nnoremap <leader>xq <cmd>TroubleToggle quickfix<cr>
"" END: trouble.nvim config


"" START: nvim-dap
nnoremap <leader>dc <cmd>lua require'dap'.continue()<CR>
nnoremap <leader>dr <cmd>lua require'dap'.repl.open()<CR>
nnoremap <leader>db <cmd>lua require'dap'.toggle_breakpoint()<CR>
nnoremap <leader>dn <cmd>lua require'dap'.step_over()<CR>
nnoremap <leader>di <cmd>lua require'dap'.step_into()<CR>
nnoremap <leader>do <cmd>lua require'dap'.step_out()<CR>
nnoremap <leader>dl <cmd>lua require'dap'.run_last()<CR>
nnoremap <leader>dp <cmd>lua require('dap.ui.widgets').preview()<CR>
nnoremap <leader>dt <cmd>lua require('dapui').toggle()<CR>
"" END: nvim-dap



"" START: true-zen.nvim key mappings
nnoremap <leader>tz :TZAtaraxis<CR>
nnoremap <leader>tm :TZMinimalist<CR>
nnoremap <leader>tf :TZFocus<CR>
nnoremap <leader>tn :TZNarrow<CR>
"" END: true-zen.nvim key mappings



"" START: buffer resize key mappings

" increase height of current window by 1 line
nnoremap <leader>wu <C-W>+
" decrease height of current window by 1 line
nnoremap <leader>wd <C-W>-
" change width of current window by 1 line to the left
nnoremap <leader>wl <C-W><
" change width of current window by 1 line to the right
nnoremap <leader>wr <C-W>>
" split two separate windows vertically across one window
nnoremap <leader>ws <C-W>=
" unsplit current window
nnoremap <leader>wx <C-W>|

"" END: buffer resize key mappings



lua << EOF
-- require statements
    require "nvim-autopairs-config"
    require "treesitter-config"
    require "nvim-treesitter.install".compilers = { 'zig', 'gcc', 'clang' }
    require "nvim-ts-autotag-config"
    require "gitsigns-config"
    require "mason-config"
    require "mason-lspconfig-config"
    require "lspconfig-config"
    require "alpha-config"
    require "indent-blankline-config"
    require "toggleterm-config"
    require "nvim-cmp-config"
    require "diffview-config"
    require "nvim-treesitter-context-config"
    require "colorizer-config"
    require "luasnip-config"
    require "trouble-config"
    require "possession-config"
    require "nvim-dap-config"
    require "nvim-dap-ui-config"
    require "nvim-dap-virtual-text-config"
    require "comment-config"
    require "true-zen-config"
    require "nvim-tree-config"
    require "bufferline-config"
    require "lualine-config"
EOF
