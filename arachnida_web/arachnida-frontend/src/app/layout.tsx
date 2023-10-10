import './globals.css';
import type { Metadata } from 'next';
import { Box } from '@mui/material';
import ThemeRegistry from '../components/theming/ThemeRegistry';

export const metadata: Metadata = {
  title: 'Arachnida III',
  description: 'Arachnida III (formerly CaveX) is a hexapod robot with 6 legs. It is a continuation of the CaveX project, which had two iterations over 2021 and 2022.',
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
	return (
		<html lang="en">
			<body>
				<ThemeRegistry>
					<Box
						component="main"
						sx={{
							flexGrow: 1,
							bgcolor: 'background.default',
						}}
					>
						{children}
					</Box>
				</ThemeRegistry>
			</body>
		</html>
	);
}
