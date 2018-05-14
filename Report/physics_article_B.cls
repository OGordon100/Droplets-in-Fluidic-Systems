\LoadClass[a4paper,11pt]{article}
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{physics_article_B}[18/05/15 - Article class for reports written by Nottingham Physics students, without biblatex]

\RequirePackage{fancyhdr}
% this package is used to provide and populate the header and footer
\RequirePackage{amsmath}
\RequirePackage{amssymb}
\RequirePackage{graphicx}
%package used to include figures as encapsulated postscript
\RequirePackage{hyperref}
\RequirePackage{appendix}
\RequirePackage{abstract}
\RequirePackage{lipsum}
\RequirePackage{float}
\RequirePackage[font=scriptsize]{caption}
\RequirePackage{caption}
\RequirePackage{subcaption}
\RequirePackage[export]{adjustbox}
\RequirePackage{color}
\RequirePackage[numbered]{mcode}
\renewcommand{\familydefault}{cmss}
\usepackage{doi}
\usepackage{pdfpages}
\usepackage[super, numbers]{natbib}
\usepackage[labelfont=bf]{caption}
\usepackage[nottoc,numbib]{tocbibind}
\usepackage[table,xcdraw]{xcolor}
\usepackage{afterpage}
\usepackage{booktabs}
\usepackage{adjustbox}
\usepackage[linesnumbered,ruled]{algorithm2e}
\usepackage{psfrag}
\DeclareMathOperator{\Tr}{Tr}
\usepackage{tikz}
\newcommand*\circled[1]{\tikz[baseline=(char.base)]{
    \node[shape=circle,draw,inner sep=2pt] (char) {#1};}}
\setlength\parindent{0pt}
\setlength{\hoffset}{-1.9mm}
\setlength{\voffset}{-0.4mm}
\setlength{\oddsidemargin}{5mm}
\setlength{\topmargin}{5mm}
\setlength{\headheight}{5mm}
\setlength{\headsep}{5mm}
\setlength{\textheight}{232mm}
\setlength{\textwidth}{160mm}
\setlength{\marginparsep}{10mm}
\setlength{\footskip}{10mm}
\usepackage{natbib}
\setlength{\bibsep}{18pt}
\hbadness = 1500
\let\endchangemargin=\endlist 

\raggedbottom

\renewcommand{\headrulewidth}{0.4pt}
\renewcommand{\footrulewidth}{0.4pt}
\renewcommand{\bibname}{References}
\renewcommand\tagform@[1]{\{\maketag@@@{\ignorespaces#1\unskip\@@italiccorr\}}}
\renewcommand{\abstractnamefont}{\normalfont\Large\bfseries}
\renewcommand{\abstracttextfont}{\normalfont\Huge}
%\renewcommand{\contentsname}{\centerline{Contents}}
\newcommand\blankpage{%
    \null
    \thispagestyle{empty}%
    \newpage}
\pagestyle{fancy}

\def\studentid#1{\gdef\@studentid{#1}}
\def\myname#1{\gdef\@myname{#1}}
\def\mytitle#1{\gdef\@mytitle{#1}}
\def\changemargin#1#2{\list{}{\rightmargin#2\leftmargin#1}\item[]}



\lhead{Student IDs: \@studentid}
\chead{}
\rhead{\@myname}
\lfoot{}
\cfoot{\thepage}
\rfoot{}


\author{\@myname \\ Student IDs: \@studentid \\ \\ School of Physics and Astronomy\\University of Nottingham}
\title{\@mytitle}