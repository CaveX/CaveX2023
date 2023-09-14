:set number 
:set relativenumber
:set autoindent
:set tabstop=4
:set shiftwidth=4
:set smarttab
:set softtabstop=4
:set mouse=a
:set shell=powershell.exe

call plug#begin()

Plug 'https://github.com/vim-airline/vim-airline'
Plug 'https://github.com/preservim/nerdtree'
Plug 'https://github.com/tpope/vim-surround'
Plug 'https://github.com/neoclide/coc.nvim'
Plug 'https://github.com/ryanoasis/vim-devicons'
Plug 'https://github.com/preservim/tagbar'
Plug 'https://github.com/terryma/vim-multiple-cursors'
Plug 'lewis6991/gitsigns.nvim'
Plug 'nvim-tree/nvim-web-devicons'
Plug 'romgrk/barbar.nvim'
Plug 'folke/tokyonight.nvim'
Plug 'nvim-treesitter/nvim-treesitter', {'do': ':TSUpdate'}
Plug 'lukas-reineke/indent-blankline.nvim'
Plug 'https://github.com/windwp/nvim-autopairs'
Plug 'https://github.com/windwp/nvim-ts-autotag'

set encoding=UTF-8

call plug#end()

colorscheme tokyonight

nnoremap <C-f> :NERDTreeFocus<CR>
nnoremap <C-n> :NerdTree<CR>
nnoremap <C-t> :NERDTreeToggle<CR>

inoremap <expr> <Tab> pumvisible() ? coc#_select_confirm() : "<Tab>"

nmap <F8> :TagbarToggle<CR>

let g:airline_powerline_fonts = 1

if !exists('g:airline_symbols')
	let g:airline_symbols = {}
endif

let g:tagbar_ctags_bin = 'C:/Users/lukap/AppData/Local/nvim/ctags58/ctags.exe'

" nvim terminal mappings
tnoremap <Esc> <C-\><C-n><CR>

" create terminal tab
nnoremap <C-A-t> :tabnew <bar> term<CR>

" barbar.nvim mappings
"" Move to previous/next
nnoremap <silent> <A-,> :BufferPrevious<CR>
nnoremap <silent> <A-.> :BufferNext<CR>

"" Re-order to previous/next
nnoremap <silent> <A-<> :BufferMovePrevious<CR>
nnoremap <silent> <A->> :BufferMoveNext<CR>

"" Go to buffer in position 1-10
nnoremap <silent> <A-1> :BufferGoto 1<CR>
nnoremap <silent> <A-2> :BufferGoto 2<CR>
nnoremap <silent> <A-3> :BufferGoto 3<CR>
nnoremap <silent> <A-4> :BufferGoto 4<CR>
nnoremap <silent> <A-5> :BufferGoto 5<CR>
nnoremap <silent> <A-6> :BufferGoto 6<CR>
nnoremap <silent> <A-7> :BufferGoto 7<CR>
nnoremap <silent> <A-8> :BufferGoto 8<CR>
nnoremap <silent> <A-9> :BufferGoto 9<CR>
nnoremap <silent> <A-0> :BufferLast<CR>

"" Pin/unpin buffer
nnoremap <silent> <A-p> :BufferPin<CR>

"" Close buffer
nnoremap <silent> <A-c> :BufferClose<CR>

"" Restore buffer
nnoremap <silent> <A-s-c> :BufferRestore<CR>

"" Magic buffer-picking mode
nnoremap <silent> <C-p> :BufferPick<CR>
" nnoremap <silent> <C-p> :BufferPickDelete<CR>

lua << EOF
-- require statements
	require("nvim-autopairs").setup {}
	require "treesitter-config"
	require "nvim-treesitter.install".compilers = { 'zig', 'gcc', 'clang' }
	require("nvim-ts-autotag").setup()
	require("gitsigns").setup()
EOF
