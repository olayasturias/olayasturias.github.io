# Exposes bibliography entries marked `selected = {true}` as site.data["timeline_papers"],
# so Liquid can merge them with news and posts on the home-page timeline.
# jekyll-scholar only renders formatted HTML, it never exposes entries as data.
require "bibtex"

module HomeTimeline
  class SelectedPapers < Jekyll::Generator
    safe true
    priority :high

    MONTHS = %w[jan feb mar apr may jun jul aug sep oct nov dec].freeze

    def generate(site)
      scholar = site.config["scholar"] || {}
      dir = (scholar["source"] || "_bibliography").sub(%r{\A/}, "")
      path = File.join(site.source, dir, scholar["bibliography"] || "references.bib")
      return unless File.exist?(path)

      entries = BibTeX.open(path).data.grep(BibTeX::Entry)
      site.data["timeline_papers"] = entries.select { |e| field(e, :selected) == "true" }.map { |e| to_item(e) }
    end

    private

    def to_item(entry)
      year = field(entry, :year).to_i
      month = month_index(field(entry, :month))
      doi = field(entry, :doi)
      {
        "kind" => "paper",
        "key" => entry.key.to_s,
        "title" => field(entry, :title),
        # Year-only entries sort to the end of their year.
        "date" => month ? Time.new(year, month, 1) : Time.new(year, 12, 31),
        "venue" => field(entry, :journal) || field(entry, :booktitle),
        "abbr" => field(entry, :abbr),
        "preview" => field(entry, :preview),
        "url" => field(entry, :html) || (doi && "https://doi.org/#{doi}"),
      }
    end

    def field(entry, name)
      return nil unless entry.has_field?(name)

      entry[name].to_s.delete("{}").strip
    end

    def month_index(value)
      return nil if value.nil? || value.empty?
      return value.to_i if value.match?(/\A\d+\z/)

      index = MONTHS.index(value[0, 3].downcase)
      index && index + 1
    end
  end
end
