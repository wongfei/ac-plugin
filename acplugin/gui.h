#pragma once

class GridView
{
public:

	ksgui_Control* _grid = nullptr;
	std::vector<ksgui_Control*> _items;
	int _rows = 0;
	int _cols = 0;

	GridView(ksgui_GUI* gui, int rows, int cols, float cellW, float cellH, float fontSize)
	{
		_rows = rows;
		_cols = cols;
		_items.resize(rows * cols);

		std::wstring name(L"grid");
		_grid = new_udt<ksgui_Control>(name, gui);
		_grid->setSize(cellW * cols, cellH * rows);
		_grid->font.reset(new_udt<Font>(eFontType::eFontMonospaced, fontSize, false, false));

		name.assign(L"");
		float x = 0, y = 0;

		for (int r = 0; r < rows; ++r) {
			y = cellH * r;
			for (int c = 0; c < cols; ++c) {

				auto item = new_udt<ksgui_Label>(&name, gui);
				item->setPosition(x + cellW * c, y);
				item->setSize(cellW, cellH);
				item->font = _grid->font;

				_grid->addControl(item);
				_items[r * cols + c] = item;
			}
		}
	}

	void setText(int row, int col, const std::wstring& text) {
		auto item = _items[row * _cols + col];
		std::wstring hack(text);
		item->setText(hack);
	}

	void setPosition(float x, float y) { _grid->setPosition(x, y); }

	ksgui_Control* getControl() { return _grid; }
};
